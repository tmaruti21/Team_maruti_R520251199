#!/usr/bin/env python3
"""
train_v2.py  –  EfficientNet-B4 multi-task lane training
=========================================================
Trains LaneSegNet (EfficientNet-B4 backbone + FPN segmentation decoder)
on your existing image-folder dataset — NO manual segmentation labels needed.

Key improvements over train.py
--------------------------------
  • Much heavier encoder : EfficientNet-B4 (~19 M params vs MobileNetV2's ~3.4 M)
  • Multi-task learning  : classification (Focal Loss) + lane segmentation (Dice+BCE)
  • Auto pseudo-masks    : lane pixels identified per-frame via HSV thresholding
                           (white/yellow lines in the lower road region)
  • Progressive resizing : Phase 1 → 256 px | Phase 2 → 320 px | Phase 3 → 384 px
  • Staged unfreezing    : head-only → top-3 EfficientNet stages → full backbone
  • Stronger augmentation: RandomAffine, RandomPerspective, GaussianBlur,
                           RandomErasing, ColorJitter, RandomGrayscale
  • Focal Loss           : down-weights easy examples, focuses on hard/rare classes
  • Seg metrics          : per-epoch binary IoU reported alongside accuracy

Requirements (same venv as before)
-----------------------------------
  pip install torch torchvision opencv-python pillow

Usage
-----
  cd src/custom_follow
  python3 train_v2.py               # default: 40 epochs

  python3 train_v2.py --epochs 60   # more epochs
  python3 train_v2.py --no-seg      # classification only (faster on CPU)
  python3 train_v2.py --seg-weight 0.3  # reduce segmentation loss contribution

Output
------
  model/lane_model_v2.pt   (new checkpoint; does NOT overwrite lane_model.pt)
"""

import argparse
import collections
import copy
import os
import random
import sys
import time

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision.transforms.functional as TF
from PIL import Image
from torch.utils.data import DataLoader, Dataset, random_split
from torchvision import transforms

# ── Make the custom_follow package importable when running as a script ────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from custom_follow.model_arch import LaneSegNet

# ── Paths and constants ───────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
DATASET_DIR = os.path.join(SCRIPT_DIR, 'dataset')
MODEL_DIR   = os.path.join(SCRIPT_DIR, 'model')
MODEL_PATH  = os.path.join(MODEL_DIR, 'lane_model_v2.pt')

IMG_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.ppm', '.bmp', '.webp', '.tif', '.tiff'}

IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD  = [0.229, 0.224, 0.225]

os.makedirs(MODEL_DIR, exist_ok=True)


# ── CLI ───────────────────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--epochs',     type=int,   default=40,
                   help='Total training epochs across all phases')
    p.add_argument('--dataset',    type=str,   default=DATASET_DIR)
    p.add_argument('--output',     type=str,   default=MODEL_PATH)
    p.add_argument('--batch',      type=int,   default=16,
                   help='Batch size (reduce if GPU OOM)')
    p.add_argument('--lr',         type=float, default=1e-3)
    p.add_argument('--seg-weight', type=float, default=0.5,
                   help='Weight of segmentation loss in combined objective')
    p.add_argument('--no-seg',     action='store_true',
                   help='Disable segmentation head (pure classification)')
    p.add_argument('--workers',    type=int,   default=4)
    p.add_argument('--val-split',  type=float, default=0.20)
    p.add_argument('--seed',       type=int,   default=42)
    return p.parse_args()


# ── Pseudo-mask generation ────────────────────────────────────────────────────

class PseudoMaskGenerator:
    """
    Generates lane ROI pseudo-masks from raw RGB PIL images using
    HSV colour thresholding — no manual annotation required.

    Detects:
      • White lane lines  : low saturation, high value
      • Yellow lane lines : hue 15-38°, saturated, bright

    Applies the mask only to the lower 55 % of the image (road region)
    and cleans up noise with morphological close → open operations.

    Tune `white_v_lo`, `yellow_h_lo/hi` and `roi_fraction` for your
    specific track/tape colour.
    """

    def __init__(
        self,
        roi_fraction: float = 0.45,
        white_v_lo:   int   = 175,
        white_s_hi:   int   = 55,
        yellow_h_lo:  int   = 15,
        yellow_h_hi:  int   = 38,
        yellow_s_lo:  int   = 70,
        yellow_v_lo:  int   = 70,
    ) -> None:
        self.roi_fraction = roi_fraction
        self._lo_white  = np.array([0,             0,          white_v_lo])
        self._hi_white  = np.array([180,            white_s_hi, 255])
        self._lo_yellow = np.array([yellow_h_lo,    yellow_s_lo, yellow_v_lo])
        self._hi_yellow = np.array([yellow_h_hi,    255,         255])
        self._close_k   = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        self._open_k    = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    def generate(self, pil_img: Image.Image) -> np.ndarray:
        """
        Return a float32 H×W array in [0, 1].
        1 = probable lane pixel, 0 = background.
        """
        rgb  = np.array(pil_img, dtype=np.uint8)
        bgr  = rgb[:, :, ::-1].copy()
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h, w = bgr.shape[:2]

        white  = cv2.inRange(hsv, self._lo_white,  self._hi_white)
        yellow = cv2.inRange(hsv, self._lo_yellow, self._hi_yellow)
        raw    = cv2.bitwise_or(white, yellow)

        # Restrict to lower road region
        mask = np.zeros_like(raw)
        roi_y = int(h * self.roi_fraction)
        mask[roi_y:] = raw[roi_y:]

        # Morphological cleanup
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._open_k)

        return mask.astype(np.float32) / 255.0


# ── Dataset ───────────────────────────────────────────────────────────────────

def _scan_dataset(root: str):
    """
    Scan a folder-per-class dataset layout.
    Returns (samples, classes, class_counts) where
      samples = list of (image_path, class_index).
    """
    classes = sorted([
        d for d in os.listdir(root)
        if os.path.isdir(os.path.join(root, d)) and any(
            os.path.splitext(f)[1].lower() in IMG_EXTENSIONS
            for f in os.listdir(os.path.join(root, d)))
    ])
    if not classes:
        raise FileNotFoundError(
            f'No image-containing sub-folders found in: {root}')

    cls_to_idx = {c: i for i, c in enumerate(classes)}
    samples    = []
    counts     = [0] * len(classes)
    for cls in classes:
        cls_dir = os.path.join(root, cls)
        for fname in os.listdir(cls_dir):
            if os.path.splitext(fname)[1].lower() in IMG_EXTENSIONS:
                samples.append((os.path.join(cls_dir, fname), cls_to_idx[cls]))
                counts[cls_to_idx[cls]] += 1
    return samples, classes, counts


def _make_transforms(img_size: int, train: bool):
    """Build (spatial_tf, color_tf) PIL→PIL transform pairs."""
    if train:
        spatial_tf = transforms.Compose([
            transforms.Resize((img_size + 64, img_size + 64), antialias=True),
            transforms.RandomCrop(img_size),
            transforms.RandomHorizontalFlip(p=0.35),
            transforms.RandomAffine(
                degrees=12,
                translate=(0.06, 0.10),
                scale=(0.82, 1.18),
                shear=(-6, 6),
                fill=0,
            ),
            transforms.RandomPerspective(distortion_scale=0.25, p=0.50, fill=0),
        ])
        color_tf = transforms.Compose([
            transforms.ColorJitter(
                brightness=0.4, contrast=0.35, saturation=0.3, hue=0.07),
            transforms.RandomAdjustSharpness(sharpness_factor=2.0, p=0.30),
            transforms.RandomGrayscale(p=0.07),
            transforms.GaussianBlur(kernel_size=5, sigma=(0.5, 2.0)),
        ])
    else:
        spatial_tf = transforms.Resize((img_size, img_size), antialias=True)
        color_tf   = transforms.Lambda(lambda x: x)   # identity
    return spatial_tf, color_tf


class LaneDatasetV2(Dataset):
    """
    Dataset that yields (image_tensor, mask_tensor, class_label).

    Augmentation order (training only):
      1. Spatial transforms → geometrically augmented PIL image
      2. PseudoMaskGenerator produces the lane mask from that image
         (mask automatically aligned with spatial transforms)
      3. Colour transforms applied to image only (NOT to mask)
      4. ToTensor + RandomErasing + Normalize
    """

    def __init__(
        self,
        samples: list,
        img_size: int,
        train: bool,
        mask_gen: PseudoMaskGenerator,
    ) -> None:
        self.samples  = samples
        self.img_size = img_size
        self.train    = train
        self.mask_gen = mask_gen
        self.spatial_tf, self.color_tf = _make_transforms(img_size, train)

        self._erase = transforms.RandomErasing(
            p=0.30, scale=(0.02, 0.10), ratio=(0.3, 3.3), value=0)

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int):
        path, label = self.samples[idx]
        img = Image.open(path).convert('RGB')

        # ── Step 1: spatial augmentation ────────────────────────────────
        img = self.spatial_tf(img)

        # ── Step 2: generate pseudo-mask from spatially-transformed image
        mask_np = self.mask_gen.generate(img)    # H×W float32

        # ── Step 3: colour augmentation (image only) ─────────────────────
        img = self.color_tf(img)

        # ── Step 4: to tensor ─────────────────────────────────────────────
        img_t  = TF.to_tensor(img)               # [3, H, W]
        if self.train:
            img_t = self._erase(img_t)           # random patch erasure
        img_t  = TF.normalize(img_t, IMAGENET_MEAN, IMAGENET_STD)

        mask_t = torch.from_numpy(mask_np).unsqueeze(0)  # [1, H, W]

        return img_t, mask_t, label

    def update_img_size(self, img_size: int) -> None:
        """Switch to a new resolution (progressive resizing)."""
        self.img_size   = img_size
        self.spatial_tf, self.color_tf = _make_transforms(img_size, self.train)


# ── Loss functions ────────────────────────────────────────────────────────────

class FocalLoss(nn.Module):
    """
    Focal Loss for multi-class classification.
    Focuses training on hard/misclassified examples by down-weighting
    well-classified samples via the modulating factor (1 - p_t)^gamma.

    Reference: Lin et al., "Focal Loss for Dense Object Detection" (2017).
    """

    def __init__(self, gamma: float = 2.0, weight=None) -> None:
        super().__init__()
        self.gamma = gamma
        if weight is not None:
            self.register_buffer('weight', weight)
        else:
            self.weight = None

    def forward(self, logits: torch.Tensor, targets: torch.Tensor) -> torch.Tensor:
        ce  = F.cross_entropy(logits, targets, weight=self.weight, reduction='none')
        pt  = torch.exp(-ce)
        return ((1.0 - pt) ** self.gamma * ce).mean()


def dice_bce_loss(
    pred:   torch.Tensor,
    target: torch.Tensor,
    eps:    float = 1e-6,
) -> torch.Tensor:
    """
    Combined Binary Cross-Entropy + Dice loss for binary segmentation.
    Both pred and target are expected in [0, 1] (pred is sigmoid-ed output).
    BCE = pixel-level supervision; Dice = region-level / shape supervision.
    """
    bce  = F.binary_cross_entropy(pred, target)

    p    = pred.view(pred.size(0), -1)
    t    = target.view(target.size(0), -1)
    inter = (p * t).sum(dim=1)
    dice  = 1.0 - (2.0 * inter + eps) / (p.sum(dim=1) + t.sum(dim=1) + eps)

    return bce + dice.mean()


# ── Per-epoch helpers ─────────────────────────────────────────────────────────

def _seg_iou(pred: torch.Tensor, target: torch.Tensor, thresh=0.5) -> float:
    """Binary IoU over a batch (threshold-based)."""
    pred_b   = (pred   >= thresh).bool()
    target_b = (target >= thresh).bool()
    inter    = (pred_b & target_b).float().sum().item()
    union    = (pred_b | target_b).float().sum().item()
    return inter / (union + 1e-6)


def run_epoch(
    model:      LaneSegNet,
    loader:     DataLoader,
    device:     torch.device,
    optimizer=None,
    focal_loss: FocalLoss | None = None,
    seg_weight: float = 0.5,
    use_seg:    bool  = True,
) -> dict:
    """
    Run one training or validation epoch.
    Returns dict with keys: loss, cls_acc, seg_iou, cls_correct (list), cls_total (list).
    """
    is_train  = optimizer is not None
    model.train(is_train)

    total_loss  = 0.0
    cls_correct = 0
    cls_seen    = 0
    iou_accum   = 0.0
    iou_batches = 0

    num_classes = model.cls_head[-1].out_features
    per_correct = [0] * num_classes
    per_total   = [0] * num_classes

    with torch.set_grad_enabled(is_train):
        for imgs, masks, labels in loader:
            imgs   = imgs.to(device)
            masks  = masks.to(device)
            labels = labels.to(device)

            cls_logits, seg_mask = model(imgs)

            # ── Loss ───────────────────────────────────────────────────
            if focal_loss is not None:
                loss_cls = focal_loss(cls_logits, labels)
            else:
                loss_cls = F.cross_entropy(cls_logits, labels)

            if use_seg:
                loss_seg = dice_bce_loss(seg_mask, masks)
                loss     = loss_cls + seg_weight * loss_seg
            else:
                loss = loss_cls

            if is_train:
                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
                optimizer.step()

            # ── Metrics ────────────────────────────────────────────────
            bs          = imgs.size(0)
            preds       = cls_logits.argmax(dim=1)
            total_loss += loss.item() * bs
            cls_correct += (preds == labels).sum().item()
            cls_seen   += bs

            for p, t in zip(preds.cpu().tolist(), labels.cpu().tolist()):
                per_total[t]   += 1
                per_correct[t] += int(p == t)

            if use_seg:
                iou_accum   += _seg_iou(seg_mask.detach(), masks)
                iou_batches += 1

    n = cls_seen
    return {
        'loss':        total_loss / n,
        'cls_acc':     cls_correct / n,
        'seg_iou':     iou_accum / max(iou_batches, 1),
        'per_correct': per_correct,
        'per_total':   per_total,
    }


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    args = _parse_args()

    torch.manual_seed(args.seed)
    random.seed(args.seed)
    np.random.seed(args.seed)

    # ── Scan dataset ─────────────────────────────────────────────────────────
    print(f'\nScanning dataset: {args.dataset}')
    samples, classes, counts = _scan_dataset(args.dataset)
    num_classes = len(classes)
    total       = len(samples)

    print(f'  Classes ({num_classes}): {classes}')
    for cls, cnt in zip(classes, counts):
        status = '✓' if cnt >= 80 else '⚠  LOW – need more images'
        print(f'    {cls:<18}: {cnt:>4}  {status}')
    print(f'  Total: {total}\n')

    if total < 100:
        print('ERROR: need at least 100 images total. Collect more data first.')
        raise SystemExit(1)

    # ── Train / val split ─────────────────────────────────────────────────────
    val_n   = max(1, int(total * args.val_split))
    train_n = total - val_n
    torch.manual_seed(args.seed)
    train_idx, val_idx = random_split(range(total), [train_n, val_n])
    train_samples = [samples[i] for i in train_idx]
    val_samples   = [samples[i] for i in val_idx]
    print(f'Train: {train_n}   Val: {val_n}')

    # ── Device ────────────────────────────────────────────────────────────────
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'Device: {device}')

    # ── Mask generator ────────────────────────────────────────────────────────
    mask_gen = PseudoMaskGenerator()

    # ── Initial datasets at Phase-1 resolution ───────────────────────────────
    PHASE1_SZ = 256
    train_ds = LaneDatasetV2(train_samples, PHASE1_SZ, train=True,  mask_gen=mask_gen)
    val_ds   = LaneDatasetV2(val_samples,   PHASE1_SZ, train=False, mask_gen=mask_gen)

    def make_loaders(ds_train, ds_val):
        tl = DataLoader(ds_train, batch_size=args.batch,
                        shuffle=True, num_workers=args.workers,
                        pin_memory=(device.type == 'cuda'))
        vl = DataLoader(ds_val,   batch_size=args.batch,
                        shuffle=False, num_workers=args.workers,
                        pin_memory=(device.type == 'cuda'))
        return tl, vl

    train_loader, val_loader = make_loaders(train_ds, val_ds)

    # ── Model ─────────────────────────────────────────────────────────────────
    print('\nBuilding LaneSegNet (EfficientNet-B4 backbone) …')
    model = LaneSegNet(num_classes=num_classes, pretrained=True).to(device)
    total_params = sum(p.numel() for p in model.parameters()) / 1e6
    print(f'  Total parameters: {total_params:.1f} M')

    # ── Class-weighted Focal Loss ─────────────────────────────────────────────
    class_weights = torch.tensor(
        [total / (num_classes * max(c, 1)) for c in counts],
        dtype=torch.float32,
    ).to(device)
    focal = FocalLoss(gamma=2.0, weight=class_weights)

    use_seg = not args.no_seg

    # ─────────────────────────────────────────────────────────────────────────
    # Training phases
    # ─────────────────────────────────────────────────────────────────────────
    # Phase 1  Head + decoder only,  img_size=256,  5 epochs
    # Phase 2  Top-3 backbone stages, img_size=320, ⌊epochs*0.45⌋ epochs
    # Phase 3  Full fine-tune,         img_size=384, remaining epochs
    # ─────────────────────────────────────────────────────────────────────────
    total_epochs = max(args.epochs, 15)
    P1 = 5
    P2 = max(5, int(total_epochs * 0.45))
    P3 = total_epochs - P1 - P2

    best_val_acc  = 0.0
    best_state    = None

    def _phase_header(phase, epochs, img_sz, mode):
        bar = '═' * 56
        print(f'\n{bar}')
        print(f'  Phase {phase}: {mode}   img_size={img_sz}   epochs={epochs}')
        print(bar)

    # ════════════════════════════════════════════════════════════════
    # Phase 1 – warm-up: heads + decoder only (frozen backbone)
    # ════════════════════════════════════════════════════════════════
    _phase_header(1, P1, PHASE1_SZ, 'heads + decoder only (backbone frozen)')
    model.freeze_backbone()

    trainable = model.trainable_params()
    optimizer = optim.AdamW(trainable, lr=args.lr, weight_decay=1e-4)
    scheduler = optim.lr_scheduler.OneCycleLR(
        optimizer, max_lr=args.lr,
        steps_per_epoch=len(train_loader), epochs=P1,
        pct_start=0.3, div_factor=10.0)

    for ep in range(1, P1 + 1):
        t0  = time.time()
        tr  = run_epoch(model, train_loader, device, optimizer, focal,
                        args.seg_weight, use_seg)
        val = run_epoch(model, val_loader,   device, None,      focal,
                        args.seg_weight, use_seg)
        scheduler.step()
        best_marker = ''
        if val['cls_acc'] > best_val_acc:
            best_val_acc = val['cls_acc']
            best_state   = copy.deepcopy(model.state_dict())
            best_marker  = '  ← best'
        print(f'  Ep {ep:02d}/{P1}  '
              f'train loss={tr["loss"]:.4f} acc={tr["cls_acc"]:.3f} '
              f'iou={tr["seg_iou"]:.3f}  |  '
              f'val acc={val["cls_acc"]:.3f} iou={val["seg_iou"]:.3f} '
              f'({time.time()-t0:.1f}s){best_marker}')

    # ════════════════════════════════════════════════════════════════
    # Phase 2 – unfreeze top-3 EfficientNet stages + increase resolution
    # ════════════════════════════════════════════════════════════════
    PHASE2_SZ = 320
    _phase_header(2, P2, PHASE2_SZ, 'top-3 backbone stages + heads/decoder')
    model.unfreeze_top_stages(n=3)

    train_ds.update_img_size(PHASE2_SZ)
    val_ds.update_img_size(PHASE2_SZ)
    train_loader, val_loader = make_loaders(train_ds, val_ds)

    trainable = model.trainable_params()
    optimizer = optim.AdamW([
        {'params': [p for n, p in model.backbone.named_parameters()
                    if any(f'features.{s}.' in n for s in range(6, 9))
                    and p.requires_grad],
         'lr': args.lr * 0.05},
        {'params': list(model.cls_head.parameters()) +
                   list(model.seg_decoder.parameters()),
         'lr': args.lr},
    ], weight_decay=1e-4)
    scheduler = optim.lr_scheduler.CosineAnnealingWarmRestarts(
        optimizer, T_0=max(P2 // 2, 5), T_mult=1, eta_min=1e-6)

    for ep in range(1, P2 + 1):
        t0  = time.time()
        tr  = run_epoch(model, train_loader, device, optimizer, focal,
                        args.seg_weight, use_seg)
        val = run_epoch(model, val_loader,   device, None,      focal,
                        args.seg_weight, use_seg)
        scheduler.step()
        best_marker = ''
        if val['cls_acc'] > best_val_acc:
            best_val_acc = val['cls_acc']
            best_state   = copy.deepcopy(model.state_dict())
            best_marker  = '  ← best'
        print(f'  Ep {ep:02d}/{P2}  '
              f'train loss={tr["loss"]:.4f} acc={tr["cls_acc"]:.3f} '
              f'iou={tr["seg_iou"]:.3f}  |  '
              f'val acc={val["cls_acc"]:.3f} iou={val["seg_iou"]:.3f} '
              f'({time.time()-t0:.1f}s){best_marker}')

    # ════════════════════════════════════════════════════════════════
    # Phase 3 – full fine-tune at max resolution
    # ════════════════════════════════════════════════════════════════
    if P3 > 0:
        PHASE3_SZ = 384
        _phase_header(3, P3, PHASE3_SZ, 'full backbone fine-tune')
        model.unfreeze_all()

        train_ds.update_img_size(PHASE3_SZ)
        val_ds.update_img_size(PHASE3_SZ)
        train_loader, val_loader = make_loaders(train_ds, val_ds)

        optimizer = optim.AdamW(model.parameters(),
                                lr=args.lr * 0.01, weight_decay=1e-4)
        scheduler = optim.lr_scheduler.CosineAnnealingLR(
            optimizer, T_max=P3, eta_min=1e-7)

        for ep in range(1, P3 + 1):
            t0  = time.time()
            tr  = run_epoch(model, train_loader, device, optimizer, focal,
                            args.seg_weight, use_seg)
            val = run_epoch(model, val_loader,   device, None,      focal,
                            args.seg_weight, use_seg)
            scheduler.step()
            best_marker = ''
            if val['cls_acc'] > best_val_acc:
                best_val_acc = val['cls_acc']
                best_state   = copy.deepcopy(model.state_dict())
                best_marker  = '  ← best'
            print(f'  Ep {ep:02d}/{P3}  '
                  f'train loss={tr["loss"]:.4f} acc={tr["cls_acc"]:.3f} '
                  f'iou={tr["seg_iou"]:.3f}  |  '
                  f'val acc={val["cls_acc"]:.3f} iou={val["seg_iou"]:.3f} '
                  f'({time.time()-t0:.1f}s){best_marker}')

    # ── Save ──────────────────────────────────────────────────────────────────
    model.load_state_dict(best_state)
    final_img_sz = 384 if P3 > 0 else (PHASE2_SZ if P2 > 0 else PHASE1_SZ)
    checkpoint = {
        'model_type':       'v2_seg',
        'model_state_dict': model.state_dict(),
        'classes':          classes,
        'img_size':         final_img_sz,
    }
    torch.save(checkpoint, args.output)
    print(f'\n✓  Model saved → {args.output}')
    print(f'   Best validation accuracy : {best_val_acc:.1%}')

    # ── Per-class accuracy on validation set ──────────────────────────────────
    model.eval()
    per_correct = [0] * num_classes
    per_total   = [0] * num_classes

    with torch.no_grad():
        for imgs, masks, labels in val_loader:
            preds = model(imgs.to(device))[0].argmax(dim=1).cpu().tolist()
            for p, t in zip(preds, labels.tolist()):
                per_total[t]   += 1
                per_correct[t] += int(p == t)

    print('\n── Per-class validation accuracy ─────────────────────────────────')
    for cls, c, tot in zip(classes, per_correct, per_total):
        acc = c / tot if tot > 0 else 0.0
        bar = '█' * int(acc * 24)
        note = '' if acc >= 0.85 else '  ⚠  collect more images'
        print(f'  {cls:<18}  {acc:.1%}  {bar}{note}')
    print('──────────────────────────────────────────────────────────────────')
    print(f'\nNext step:')
    print(f'  ros2 run custom_follow infer_node \\')
    print(f'    --ros-args -p model_path:={args.output}')


if __name__ == '__main__':
    main()
