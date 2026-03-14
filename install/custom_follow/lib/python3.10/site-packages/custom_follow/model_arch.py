#!/usr/bin/env python3
"""
model_arch.py  –  LaneSegNet: EfficientNet-B4 + FPN segmentation decoder
=========================================================================
Shared by train_v2.py (training) and infer_node.py (inference).

Architecture
------------
  Encoder : EfficientNet-B4 pretrained on ImageNet (~19 M params)
            Features tapped at three scales:
              c3  –  56 ch  @ H/8  × W/8   (fine detail / texture)
              c5  –  160 ch @ H/16 × W/16  (mid-level structure)
              c8  –  1792 ch @ H/32 × W/32 (high-level semantics)

  Classification head
      AdaptiveAvgPool(c8) → Flatten → Dropout(0.4) → Linear(1792, C)

  Segmentation decoder  (FPN-style, no supervision needed at label level)
      lateral_c8 : 1792 → 256  (1×1 CBR)
      lateral_c5 :  160 → 256  (1×1 CBR)
      lateral_c3 :   56 → 128  (1×1 CBR)
      merge_p5   : cat(up_c8=256, lat_c5=256) 512 → 256  (3×3 CBR)
      merge_p3   : cat(up_p5=256, lat_c3=128) 384 → 128  (3×3 CBR)
      Bilinear upsample to input H×W  →  Conv(1) → Sigmoid  lane mask

Checkpoint format (written by train_v2.py)
------------------------------------------
  {
      'model_type'      : 'v2_seg',
      'model_state_dict': <state_dict>,
      'classes'         : ['dead_end', 'intersection', 'left', ...],
      'img_size'        : 384,
  }
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import efficientnet_b4, EfficientNet_B4_Weights
from torchvision.models.feature_extraction import create_feature_extractor

__all__ = ['LaneSegNet', 'build_lane_seg_net']


# ── Internal helpers ──────────────────────────────────────────────────────────

def _cbr(in_ch: int, out_ch: int, k: int = 3) -> nn.Sequential:
    """Conv → BatchNorm → ReLU  (same-spatial padding for k=1,3)."""
    return nn.Sequential(
        nn.Conv2d(in_ch, out_ch, k, padding=k // 2, bias=False),
        nn.BatchNorm2d(out_ch),
        nn.ReLU(inplace=True),
    )


# ── FPN segmentation decoder ──────────────────────────────────────────────────

class _SegDecoder(nn.Module):
    """
    FPN-style lane-mask decoder.

    Fuses EfficientNet-B4 features at three resolutions:
      c8 (1792 ch, H/32)  c5 (160 ch, H/16)  c3 (56 ch, H/8)

    Produces a 1-channel sigmoid lane probability map at the original
    input resolution (no strided operations → preserves fine lane detail).
    """

    def __init__(self) -> None:
        super().__init__()

        # ── Lateral channel-reduction (1×1 convs) ────────────────────────
        self.lat_c8 = _cbr(1792, 256, k=1)
        self.lat_c5 = _cbr(160,  256, k=1)
        self.lat_c3 = _cbr(56,   128, k=1)

        # ── Merge convolutions after top-down feature fusion ──────────────
        self.merge_p5 = _cbr(512, 256)   # cat(up_c8=256, lat_c5=256) → 256
        self.merge_p3 = _cbr(384, 128)   # cat(up_p5=256, lat_c3=128) → 128

        # ── Additional 3×3 refine before final head ───────────────────────
        self.refine = _cbr(128, 128)

        # ── 1×1 lane logit head ───────────────────────────────────────────
        self.out_conv = nn.Conv2d(128, 1, kernel_size=1)

    def forward(
        self,
        c3: torch.Tensor,
        c5: torch.Tensor,
        c8: torch.Tensor,
        target_hw: tuple[int, int],
    ) -> torch.Tensor:

        # ── Top-down path ─────────────────────────────────────────────────
        p8 = self.lat_c8(c8)                                         # B,256,H/32
        p8_up = F.interpolate(p8, size=c5.shape[-2:],
                              mode='bilinear', align_corners=False)  # → H/16

        lat5 = self.lat_c5(c5)
        p5 = self.merge_p5(torch.cat([p8_up, lat5], dim=1))         # B,256,H/16
        p5_up = F.interpolate(p5, size=c3.shape[-2:],
                              mode='bilinear', align_corners=False)  # → H/8

        lat3 = self.lat_c3(c3)
        p3 = self.merge_p3(torch.cat([p5_up, lat3], dim=1))         # B,128,H/8
        p3 = self.refine(p3)

        # ── Upsample to original input resolution ─────────────────────────
        p_full = F.interpolate(p3, size=target_hw,
                               mode='bilinear', align_corners=False)
        return torch.sigmoid(self.out_conv(p_full))                  # B,1,H,W


# ── Public model class ────────────────────────────────────────────────────────

class LaneSegNet(nn.Module):
    """
    Dual-output lane model using EfficientNet-B4 as encoder.

    forward(x) returns a 2-tuple:
        cls_logits : FloatTensor[B, num_classes]   raw classification scores
        seg_mask   : FloatTensor[B, 1, H, W]       lane probability in [0, 1]

    The segmentation output is an auxiliary output — the primary navigation
    signal comes from cls_logits, but the mask gives spatial interpretability
    and improves feature learning through multi-task training.
    """

    _C8_CH = 1792   # EfficientNet-B4 final feature channels (before avgpool)

    def __init__(self, num_classes: int, pretrained: bool = True) -> None:
        super().__init__()

        weights = EfficientNet_B4_Weights.IMAGENET1K_V1 if pretrained else None
        base    = efficientnet_b4(weights=weights)

        # Tap features at three scales; avgpool + classifier from base discarded
        self.backbone = create_feature_extractor(base, return_nodes={
            'features.3': 'c3',   # 56 ch,   H/8
            'features.5': 'c5',   # 160 ch,  H/16
            'features.8': 'c8',   # 1792 ch, H/32
        })

        # ── Classification branch ─────────────────────────────────────────
        self.cls_pool = nn.AdaptiveAvgPool2d((1, 1))
        self.cls_head = nn.Sequential(
            nn.Dropout(p=0.4),
            nn.Linear(self._C8_CH, num_classes),
        )

        # ── Segmentation branch ───────────────────────────────────────────
        self.seg_decoder = _SegDecoder()

    # ── Forward pass ─────────────────────────────────────────────────────────

    def forward(self, x: torch.Tensor):
        target_hw = (x.shape[-2], x.shape[-1])
        feats = self.backbone(x)
        c3 = feats['c3']
        c5 = feats['c5']
        c8 = feats['c8']

        # Classification
        cls = self.cls_pool(c8).flatten(1)
        cls = self.cls_head(cls)

        # Segmentation
        seg = self.seg_decoder(c3, c5, c8, target_hw)

        return cls, seg

    # ── Training utilities ────────────────────────────────────────────────────

    def freeze_backbone(self) -> None:
        """Freeze all backbone parameters (Phase-1 head-warm-up)."""
        for p in self.backbone.parameters():
            p.requires_grad = False

    def unfreeze_top_stages(self, n: int = 3) -> None:
        """Unfreeze the last *n* EfficientNet-B4 feature stages (0-indexed: 0-8)."""
        first = 9 - n   # e.g. n=3 → stages 6,7,8
        for name, p in self.backbone.named_parameters():
            for s in range(first, 9):
                if name.startswith(f'features.{s}.'):
                    p.requires_grad = True
                    break

    def unfreeze_all(self) -> None:
        """Unfreeze every parameter (Phase-3 full fine-tune)."""
        for p in self.backbone.parameters():
            p.requires_grad = True

    def trainable_params(self):
        """Return only the parameters with requires_grad=True."""
        return [p for p in self.parameters() if p.requires_grad]


# ── Convenience factory ───────────────────────────────────────────────────────

def build_lane_seg_net(
    num_classes: int,
    pretrained: bool = True,
    checkpoint_path: str | None = None,
) -> LaneSegNet:
    """Build LaneSegNet, optionally loading weights from a v2_seg checkpoint."""
    model = LaneSegNet(num_classes=num_classes, pretrained=pretrained)
    if checkpoint_path is not None:
        ckpt = torch.load(checkpoint_path, map_location='cpu')
        model.load_state_dict(ckpt['model_state_dict'])
    return model


def _main() -> None:
    """Run a quick architecture smoke test when executed as a script."""
    import argparse

    parser = argparse.ArgumentParser(
        description=(
            'LaneSegNet architecture module. This file is normally imported by '
            'train_v2.py and infer_node.py.'
        )
    )
    parser.add_argument('--num-classes', type=int, default=7)
    parser.add_argument('--img-size', type=int, default=384)
    parser.add_argument('--pretrained', action='store_true')
    parser.add_argument('--device', choices=['cpu', 'cuda'], default='cpu')
    args = parser.parse_args()

    if args.device == 'cuda' and not torch.cuda.is_available():
        print('CUDA requested but not available; falling back to CPU.')
        args.device = 'cpu'

    device = torch.device(args.device)
    model = LaneSegNet(num_classes=args.num_classes, pretrained=args.pretrained).to(device)
    model.eval()

    dummy = torch.randn(1, 3, args.img_size, args.img_size, device=device)
    with torch.no_grad():
        cls_logits, seg_mask = model(dummy)

    total_params = sum(p.numel() for p in model.parameters())
    print('LaneSegNet smoke test passed')
    print(f'  device      : {device}')
    print(f'  cls_logits  : {tuple(cls_logits.shape)}')
    print(f'  seg_mask    : {tuple(seg_mask.shape)}')
    print(f'  parameters  : {total_params}')


if __name__ == '__main__':
    _main()
