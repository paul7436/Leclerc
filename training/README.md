# Training a custom target detector

The pretrained COCO model used by default knows `sports ball`, `bottle` or
`cup`, but not a balloon, a cardboard target or your small robot. This folder
explains how to train a YOLO model on your own targets with
[Ultralytics](https://docs.ultralytics.com/). No dataset is included in the
repository: you photograph and label your own.

## Dataset layout

Ultralytics expects images and labels in parallel folders, split into a
training and a validation set:

```text
training/
`-- datasets/
    `-- turret_targets/
        |-- dataset.yaml
        |-- images/
        |   |-- train/          about 80 percent of the images
        |   |   |-- img_0001.jpg
        |   |   `-- ...
        |   `-- val/            about 20 percent of the images
        |       `-- ...
        `-- labels/
            |-- train/
            |   |-- img_0001.txt
            |   `-- ...
            `-- val/
                `-- ...
```

`training/datasets/` is ignored by git, so your photos never end up in the
repository.

Each image has a label file with the same base name. Each line of a label
file describes one object, in normalized coordinates (0 to 1, relative to the
image width and height):

```text
<class_id> <x_center> <y_center> <width> <height>
```

For example, a balloon (class 0) in the middle of the image, a quarter of the
image wide and a third of it high:

```text
0 0.500 0.500 0.250 0.333
```

An image with no target gets an empty label file (or no label file at all).
Such background images teach the model what is not a target and reduce false
detections; aim for about 10 percent of the dataset.

## dataset.yaml

Copy [dataset.example.yaml](dataset.example.yaml) to
`datasets/turret_targets/dataset.yaml` and edit the class names:

```yaml
path: datasets/turret_targets   # dataset root, relative to where you run train.py
train: images/train
val: images/val
names:
  0: balloon
  1: cardboard_target
  2: robot
```

The class names are what you later list in `detection.target_classes` in
`host/config.yaml`.

## Collecting and labelling images

- Take the photos **with the turret's own camera**, at the resolution set in
  `host/config.yaml`, from the distances and angles you will shoot from.
- Vary lighting, backgrounds, target positions and partial occlusions.
  A few hundred labelled instances per class is a good start.
- Label with any tool that exports the YOLO format, for example
  [Label Studio](https://labelstud.io/) or [CVAT](https://www.cvat.ai/).
- Keep images of the same scene in the same split, so validation measures
  how well the model generalizes rather than how well it remembers.
- Only label inert targets. People and pets are protected classes: the host
  never treats them as targets and refuses a configuration that lists them in
  `detection.target_classes`. While a custom model is in use, the host keeps
  running the COCO guard model (`safety.guard_model_path`) on every frame so
  people and pets in view still veto firing.

## Training

From this folder, with the host requirements installed:

```bash
python train.py --data datasets/turret_targets/dataset.yaml
python train.py --data datasets/turret_targets/dataset.yaml --epochs 150 --imgsz 640 --device 0
```

`train.py` starts from the small pretrained `yolo11n.pt` (transfer learning),
which trains quickly and runs in real time on a laptop CPU or a Raspberry Pi
5. Use `--model yolo11s.pt` for more accuracy at a lower frame rate.

Training writes its results to `runs/detect/<name>/`. Look at the curves and
the validation images there, then copy the best weights next to the host:

```bash
cp runs/detect/turret_targets/weights/best.pt ../host/targets.pt
```

and update `host/config.yaml`:

```yaml
detection:
  model_path: targets.pt
  target_classes: [balloon, cardboard_target, robot]
```

The host checks at startup that every class in `target_classes` exists in the
model and that a model able to detect the protected classes is loaded.
