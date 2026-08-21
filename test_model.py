"""
RoboSort Model Test Script
--------------------------
Loads the trained YOLO model and runs inference on test images (or webcam),
mapping the 9 dataset classes to bio / non-bio for RoboSort's sorting logic.

Usage:
    python test_model.py                        # run on test images
    python test_model.py --source 0             # webcam
    python test_model.py --source path/to/img  # single image or folder
    python test_model.py --model path/to/model.pt
    python test_model.py --conf 0.4 --save     # save annotated output
"""

import argparse
from pathlib import Path
import cv2
from ultralytics import YOLO

#  Class mapping 
CLASS_TO_CATEGORY = {
    "Polystyrene": "non-bio",
    "bread":       "bio",
    "cardboard":   "bio",
    "eggs":        "bio",
    "metal":       "non-bio",
    "paper":       "bio",
    "peels":       "bio",
    "plastic":     "non-bio",
    "walnuts":     "bio",
    
    "banana":       "bio",    "apple":      "bio",    "sandwich":  "bio",
    "orange":       "bio",    "broccoli":   "bio",    "carrot":    "bio",
    "hot dog":      "bio",    "pizza":      "bio",    "donut":     "bio",
    "cake":         "bio",    "bird":       "bio",    "cat":       "bio",
    "dog":          "bio",    "horse":      "bio",    "sheep":     "bio",
    "cow":          "bio",    "elephant":   "bio",    "bear":      "bio",
    "zebra":        "bio",    "giraffe":    "bio",
    "bottle":       "non-bio", "wine glass": "non-bio", "cup":      "non-bio",
    "fork":         "non-bio", "knife":     "non-bio", "spoon":    "non-bio",
    "bowl":         "non-bio", "chair":     "non-bio", "couch":    "non-bio",
    "bed":          "non-bio", "toilet":    "non-bio", "tv":       "non-bio",
    "laptop":       "non-bio", "mouse":     "non-bio", "remote":   "non-bio",
    "keyboard":     "non-bio", "cell phone":"non-bio", "microwave":"non-bio",
    "oven":         "non-bio", "toaster":   "non-bio", "sink":     "non-bio",
    "refrigerator": "non-bio", "clock":     "non-bio", "vase":     "non-bio",
    "scissors":     "non-bio", "toothbrush":"non-bio", "backpack": "non-bio",
    "umbrella":     "non-bio", "handbag":   "non-bio", "tie":      "non-bio",
    "suitcase":     "non-bio", "frisbee":   "non-bio", "skis":     "non-bio",
    "snowboard":    "non-bio", "sports ball":"non-bio","kite":     "non-bio",
    "baseball bat": "non-bio","baseball glove":"non-bio","skateboard":"non-bio",
    "surfboard":    "non-bio","tennis racket":"non-bio","bicycle":  "non-bio",
    "car":          "non-bio", "motorcycle":"non-bio", "airplane": "non-bio",
    "bus":          "non-bio", "train":     "non-bio", "truck":    "non-bio",
    "boat":         "non-bio",
}

CUSTOM_CLASSES = {
    "Polystyrene", "bread", "cardboard", "eggs",
    "metal", "paper", "peels", "plastic", "walnuts",
}

CATEGORY_COLOR = {
    "bio":     (34, 197, 94),   # green
    "non-bio": (239, 68, 68),   # red
    "unknown": (156, 163, 175), # gray
}

DEFAULT_MODEL  = "data/yolov11.pt"
DEFAULT_SOURCE = "data/test/images"
DEFAULT_CONF   = 0.25
DEFAULT_IOU    = 0.45
IMG_SIZE       = 320  # must match training imgsz


def get_category(class_name: str) -> str:
    return CLASS_TO_CATEGORY.get(class_name, "unknown")


def draw_detections(frame, results, class_names):
    boxes = results.boxes
    for box in boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf            = float(box.conf[0])
        cls_idx         = int(box.cls[0])
        class_name      = class_names[cls_idx]
        category        = get_category(class_name)
        color           = CATEGORY_COLOR[category]

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        label = f"{category} ({class_name}) {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
        cv2.putText(
            frame, label, (x1 + 2, y1 - 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA,
        )
    return frame


def print_detection_summary(class_names, all_detections: list[dict]):
    if not all_detections:
        print("No detections found.")
        return

    bio_count     = sum(1 for d in all_detections if d["category"] == "bio")
    nonbio_count  = sum(1 for d in all_detections if d["category"] == "non-bio")
    unknown_count = sum(1 for d in all_detections if d["category"] == "unknown")

    print("\n" + "=" * 50)
    print("  DETECTION SUMMARY")
    print("=" * 50)
    print(f"  Total detections : {len(all_detections)}")
    print(f"  Bio              : {bio_count}")
    print(f"  Non-bio          : {nonbio_count}")
    if unknown_count:
        print(f"  Unknown          : {unknown_count}")

    # Per-class breakdown
    from collections import Counter
    class_counts = Counter(d["class_name"] for d in all_detections)
    print("\n  Per class:")
    for cls, count in sorted(class_counts.items()):
        cat = get_category(cls)
        print(f"    {cls:<14} → {cat:<8}  ({count} detection{'s' if count != 1 else ''})")
    print("=" * 50)


def run_on_images(model, source_path: Path, conf: float, iou: float,
                  save: bool, save_dir: Path):
    image_exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    images = sorted(
        p for p in source_path.iterdir()
        if p.suffix.lower() in image_exts
    ) if source_path.is_dir() else [source_path]

    if not images:
        print(f"No images found at {source_path}")
        return

    all_detections = []
    class_names    = model.names

    print(f"\nRunning inference on {len(images)} image(s)...")
    print("Press any key to advance, 'q' to quit.\n")

    for img_path in images:
        frame = cv2.imread(str(img_path))
        if frame is None:
            print(f"  [skip] Cannot read {img_path.name}")
            continue

        results = model(frame, conf=conf, iou=iou, imgsz=IMG_SIZE, verbose=False)[0]

        # Collect detections for summary
        for box in results.boxes:
            cls_name = class_names[int(box.cls[0])]
            all_detections.append({
                "image":     img_path.name,
                "class_name": cls_name,
                "category":  get_category(cls_name),
                "conf":      float(box.conf[0]),
            })

        annotated = draw_detections(frame.copy(), results, class_names)

        # Top-left status line
        cat_labels = [get_category(class_names[int(b.cls[0])]) for b in results.boxes]
        status = "bio" if "bio" in cat_labels and "non-bio" not in cat_labels else \
                 "non-bio" if "non-bio" in cat_labels and "bio" not in cat_labels else \
                 "mixed" if cat_labels else "none"
        cv2.putText(
            annotated, f"File: {img_path.name}  |  Status: {status}",
            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA,
        )

        if save:
            out_path = save_dir / img_path.name
            cv2.imwrite(str(out_path), annotated)

        cv2.imshow("RoboSort - Model Test", annotated)
        key = cv2.waitKey(0) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    print_detection_summary(class_names, all_detections)


def run_on_webcam(model, device_id: int, conf: float, iou: float):
    cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        print(f"Cannot open webcam {device_id}")
        return

    class_names = model.names
    print("Webcam feed — press 'q' to quit.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results   = model(frame, conf=conf, iou=iou, imgsz=IMG_SIZE, verbose=False)[0]
        annotated = draw_detections(frame.copy(), results, class_names)

        cv2.imshow("RoboSort - Live Test", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="RoboSort YOLO model tester")
    parser.add_argument("--model",  default=DEFAULT_MODEL,  help="Path to .pt model file")
    parser.add_argument("--source", default=DEFAULT_SOURCE, help="Image/folder path or webcam index (0, 1, ...)")
    parser.add_argument("--conf",   type=float, default=DEFAULT_CONF, help="Confidence threshold")
    parser.add_argument("--iou",    type=float, default=DEFAULT_IOU,  help="IoU threshold for NMS")
    parser.add_argument("--save",   action="store_true", help="Save annotated images to output/")
    args = parser.parse_args()

    # Load model
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"Model not found: {model_path}")
        return
    print(f"Loading model: {model_path}")
    model = YOLO(str(model_path))
    class_list = list(model.names.values())
    is_custom = set(class_list) <= CUSTOM_CLASSES or any(c in CUSTOM_CLASSES for c in class_list)
    if not is_custom:
        print("WARNING: Model appears to be the base COCO model, not the custom-trained model.")
        print("         Train the model first, then point --model to runs/train/.../weights/best.pt")
        print("         Running anyway using COCO → bio/non-bio mapping.\n")
    print(f"Classes ({len(class_list)}): {class_list}\n")

    # Save dir
    save_dir = Path("output")
    if args.save:
        save_dir.mkdir(exist_ok=True)
        print(f"Annotated images will be saved to: {save_dir}/")

    # Source: webcam or file/folder
    if args.source.isdigit():
        run_on_webcam(model, int(args.source), args.conf, args.iou)
    else:
        source_path = Path(args.source)
        if not source_path.exists():
            print(f"Source not found: {source_path}")
            return
        run_on_images(model, source_path, args.conf, args.iou, args.save, save_dir)


if __name__ == "__main__":
    main()
