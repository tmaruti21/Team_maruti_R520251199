#!/usr/bin/env python3
"""
train.py  –  Fine-tune MobileNetV2 on your lane dataset
=========================================================
Run this on your laptop (or any machine with the dataset).
GPU is optional — CPU training on ~900 images takes ~10–20 minutes.

Requirements
------------
  pip install torch torchvision pillow

Dataset layout expected (inside this script's directory):
  dataset/
      straight/       *.jpg / *.png
      left/
      right/
      intersection/
      dead_end/

What this script does
---------------------
  1. Loads all images, splits into 80 % train / 20 % validation
  2. Applies data augmentation (flip, colour jitter, rotation) to training set
     → this is the key to generalising across environments
  3. Fine-tunes MobileNetV2 (pretrained on ImageNet, 1000 classes)
     – Phase 1: freeze all base layers, train only the new head (5 epochs)
     – Phase 2: unfreeze top layers, fine-tune end-to-end (25 epochs)
  4. Saves the best model to  model/lane_model.pt
  5. Prints per-class accuracy so you know which classes need more data

Usage
-----
  cd src/custom_follow
  python3 train.py

  # Train for more epochs:
  python3 train.py --epochs 40

  # Use a different dataset path:
  python3 train.py --dataset /path/to/dataset
"""

import argparse
import os
import time
import copy

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
from torchvision import datasets, models, transforms
from torchvision.models import MobileNet_V2_Weights

# ── Config ────────────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
DATASET_DIR  = os.path.join(SCRIPT_DIR, 'dataset')
MODEL_DIR    = os.path.join(SCRIPT_DIR, 'model')
MODEL_PATH   = os.path.join(MODEL_DIR, 'lane_model.pt')
IMG_SIZE     = 224   # MobileNetV2 input
BATCH_SIZE   = 32
VAL_SPLIT    = 0.20  # 20 % held out for validation
SEED         = 42

os.makedirs(MODEL_DIR, exist_ok=True)

# ── Argument parsing ──────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--epochs',  type=int,   default=30,
                    help='Total fine-tuning epochs (default: 30)')
parser.add_argument('--dataset', type=str,   default=DATASET_DIR,
                    help='Path to dataset root')
parser.add_argument('--lr',      type=float, default=1e-3,
                    help='Initial learning rate for head (default: 1e-3)')
args = parser.parse_args()


# ── Transforms ───────────────────────────────────────────────────────────────
# Augmentation helps the model generalise to new environments.
train_tf = transforms.Compose([
    transforms.Resize((IMG_SIZE + 32, IMG_SIZE + 32)),
    transforms.RandomCrop(IMG_SIZE),
    transforms.RandomHorizontalFlip(p=0.3),      # lanes can be mirrored
    transforms.RandomRotation(degrees=8),         # slight camera tilt
    transforms.ColorJitter(
        brightness=0.3,   # different lighting conditions
        contrast=0.3,
        saturation=0.2,
        hue=0.05,
    ),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],   # ImageNet stats
                         std =[0.229, 0.224, 0.225]),
])

val_tf = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std =[0.229, 0.224, 0.225]),
])


# ── Load dataset ──────────────────────────────────────────────────────────────
print(f'\nLoading dataset from: {args.dataset}')

_IMG_EXTS = {'.jpg', '.jpeg', '.png', '.ppm', '.bmp', '.pgm', '.tif', '.tiff', '.webp'}

class NonEmptyImageFolder(datasets.ImageFolder):
    """ImageFolder that silently skips class folders with no images."""
    def find_classes(self, directory):
        classes = sorted([
            d for d in os.listdir(directory)
            if os.path.isdir(os.path.join(directory, d))
            and any(os.path.splitext(f)[1].lower() in _IMG_EXTS
                    for f in os.listdir(os.path.join(directory, d)))
        ])
        if not classes:
            raise FileNotFoundError(
                f'No images found in any subfolder of {directory}\n'
                f'  Collect images first:  python3 collect_data.py')
        return classes, {c: i for i, c in enumerate(classes)}

full_dataset = NonEmptyImageFolder(args.dataset, transform=train_tf)
classes      = full_dataset.classes
num_classes  = len(classes)
print(f'  Classes ({num_classes}): {classes}')
print(f'  (Empty folders skipped – add images later and retrain)')

# Count per class
class_counts = [0] * num_classes
for _, label in full_dataset.samples:
    class_counts[label] += 1
for cls, cnt in zip(classes, class_counts):
    status = '✓' if cnt >= 80 else '⚠ LOW – add more images'
    print(f'  {cls:<14} : {cnt:>4}  {status}')

total = sum(class_counts)
print(f'  Total          : {total}\n')

if total < 100:
    print('ERROR: Too few images. Collect at least 80 per class before training.')
    raise SystemExit(1)

# Split train / val
torch.manual_seed(SEED)
val_size   = int(total * VAL_SPLIT)
train_size = total - val_size
train_ds, val_ds = random_split(full_dataset, [train_size, val_size])

# Apply val transform to validation split
val_ds.dataset = copy.deepcopy(full_dataset)
val_ds.dataset.transform = val_tf

train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True,  num_workers=2)
val_loader   = DataLoader(val_ds,   batch_size=BATCH_SIZE, shuffle=False, num_workers=2)
print(f'Train: {train_size} images   Validation: {val_size} images')


# ── Model ─────────────────────────────────────────────────────────────────────
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f'Using device: {device}\n')

model = models.mobilenet_v2(weights=MobileNet_V2_Weights.IMAGENET1K_V1)

# Replace classifier head for our number of classes
in_features = model.classifier[1].in_features
model.classifier = nn.Sequential(
    nn.Dropout(0.2),
    nn.Linear(in_features, num_classes),
)
model = model.to(device)

criterion = nn.CrossEntropyLoss()


# ── Compute class weights (handles imbalanced classes) ────────────────────────
weights = torch.tensor(
    [total / (num_classes * cnt) for cnt in class_counts],
    dtype=torch.float32,
).to(device)
criterion = nn.CrossEntropyLoss(weight=weights)


# ── Training helper ───────────────────────────────────────────────────────────

def run_epoch(loader, train_mode):
    model.train(train_mode)
    total_loss = correct = seen = 0
    with torch.set_grad_enabled(train_mode):
        for imgs, labels in loader:
            imgs, labels = imgs.to(device), labels.to(device)
            if train_mode:
                optimizer.zero_grad()
            outputs = model(imgs)
            loss    = criterion(outputs, labels)
            if train_mode:
                loss.backward()
                optimizer.step()
            total_loss += loss.item() * imgs.size(0)
            correct    += (outputs.argmax(1) == labels).sum().item()
            seen       += imgs.size(0)
    return total_loss / seen, correct / seen


# ── Phase 1: train head only ──────────────────────────────────────────────────
for param in model.features.parameters():
    param.requires_grad = False

HEAD_EPOCHS = min(5, args.epochs // 4)
optimizer   = optim.Adam(model.classifier.parameters(), lr=args.lr)
scheduler   = optim.lr_scheduler.StepLR(optimizer, step_size=3, gamma=0.5)

print(f'── Phase 1: head only ({HEAD_EPOCHS} epochs) ─────────────────────')
for epoch in range(HEAD_EPOCHS):
    t0             = time.time()
    train_loss, train_acc = run_epoch(train_loader, True)
    val_loss,   val_acc   = run_epoch(val_loader,   False)
    scheduler.step()
    print(f'  Ep {epoch+1:02d}/{HEAD_EPOCHS}  '
          f'train loss={train_loss:.4f} acc={train_acc:.3f}  '
          f'val loss={val_loss:.4f} acc={val_acc:.3f}  '
          f'({time.time()-t0:.1f}s)')


# ── Phase 2: unfreeze top layers and fine-tune ───────────────────────────────
for param in model.features[-4:].parameters():   # unfreeze last 4 conv blocks
    param.requires_grad = True

FT_EPOCHS = args.epochs - HEAD_EPOCHS
optimizer  = optim.Adam(
    [{'params': model.features[-4:].parameters(), 'lr': args.lr * 0.1},
     {'params': model.classifier.parameters(),    'lr': args.lr}],
)
scheduler  = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=FT_EPOCHS)

best_val_acc  = 0.0
best_state    = None

print(f'\n── Phase 2: fine-tune top layers ({FT_EPOCHS} epochs) ──────────────')
for epoch in range(FT_EPOCHS):
    t0             = time.time()
    train_loss, train_acc = run_epoch(train_loader, True)
    val_loss,   val_acc   = run_epoch(val_loader,   False)
    scheduler.step()

    marker = ''
    if val_acc > best_val_acc:
        best_val_acc = val_acc
        best_state   = copy.deepcopy(model.state_dict())
        marker       = '  ← best'

    print(f'  Ep {epoch+1:02d}/{FT_EPOCHS}  '
          f'train loss={train_loss:.4f} acc={train_acc:.3f}  '
          f'val loss={val_loss:.4f} acc={val_acc:.3f}  '
          f'({time.time()-t0:.1f}s){marker}')


# ── Save best model ───────────────────────────────────────────────────────────
model.load_state_dict(best_state)

torch.save({
    'model_state_dict': model.state_dict(),
    'classes':          classes,
    'img_size':         IMG_SIZE,
}, MODEL_PATH)

print(f'\n✓  Model saved → {MODEL_PATH}')
print(f'   Best validation accuracy: {best_val_acc:.1%}')


# ── Per-class accuracy on validation set ─────────────────────────────────────
model.eval()
class_correct = [0] * num_classes
class_total   = [0] * num_classes

with torch.no_grad():
    for imgs, labels in val_loader:
        imgs, labels = imgs.to(device), labels.to(device)
        outputs = model(imgs)
        preds   = outputs.argmax(1)
        for p, t in zip(preds, labels):
            class_total[t]   += 1
            class_correct[t] += int(p == t)

print('\n── Per-class validation accuracy ───────────────────────')
for cls, correct, total in zip(classes, class_correct, class_total):
    acc = correct / total if total > 0 else 0.0
    bar = '█' * int(acc * 20)
    tip = '' if acc >= 0.85 else '  ⚠ collect more images for this class'
    print(f'  {cls:<14} {acc:.1%}  {bar}{tip}')
print('────────────────────────────────────────────────────────')
print('  Next step: deploy with  ros2 run custom_follow infer_node')
