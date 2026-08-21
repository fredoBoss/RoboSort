RoboSort - Setup TODO
Last updated: 2026-08-21

Legend:  [x] done   [ ] todo   [!] blocker / decision needed


================================================================
DONE
================================================================
[x] Orange Pi One identified (Allwinner H3, 512 MB, Ethernet-only, SD-boot only)
[x] Armbian 26.5.1 minimal downloaded + SHA256 verified
    c0a3c8f47e20cca6d89474ddcf48574f2bac12e78a0cc14b22790713fa47a00e
[x] microSD (62.5 GB) wiped and flashed with Armbian
[x] Flash verified by read-back: 1,312,817,152 bytes, hash matched
[x] CV pipeline written on laptop: capture / classify / serial_link / vision
[x] Dataset extracted to YOLO layout (data/train, data/test - images + labels)


================================================================
PHASE 1 - GET THE PI BOOTING AND ON THE NETWORK
================================================================
[ ] Power the Pi from the 4.0x1.7 mm BARREL JACK (5V/2A).
    Do NOT power from micro-USB - it is OTG-only and browns out the board.
    This is the #1 cause of an apparently dead Orange Pi One.
[ ] Insert flashed microSD, connect Ethernet cable to the router.
[ ] Find the Pi on the LAN. PC is 192.168.1.7/24, gateway 192.168.1.1.
        nmap -sn 192.168.1.0/24
    or check the router's DHCP client list for "orangepione".
[ ] Fallback if it never appears: UART debug console, 3-pin header,
    115200 8N1, 3.3V. Only way in with no network.
[ ] First login: root / 1234 (forces an immediate password change).
[ ] Complete armbian-firstlogin: set root password, create normal user, locale.
[ ] Set a stable address - DHCP reservation on the router, or static IP.
    Record the final IP here: ______________
[ ] Confirm SSH works from the laptop:  ssh <user>@<pi-ip>
[ ] apt update && apt full-upgrade
[ ] Enable zram / swap. 512 MB RAM is very tight for any vision workload.


================================================================
PHASE 2 - RUNTIME ON THE PI
================================================================
[!] DECISION: requirements.txt is currently unusable on this board.
    It pins ultralytics==8.3.40, which pulls in PyTorch. There is no
    usable armhf PyTorch wheel for a 32-bit Cortex-A7, and CLAUDE.md
    already states: "Target TFLite Runtime or ONNX Runtime - never full
    TensorFlow/PyTorch."
    Pick the Pi-side runtime BEFORE installing anything:
      - NCNN      (train.py already exports to it; best ARM CPU perf)
      - ONNX Runtime  (check an armhf wheel actually exists first)
      - TFLite Runtime
    Then split requirements.txt into requirements-pc.txt (training,
    ultralytics) and requirements-pi.txt (inference only).

[ ] Install base packages on the Pi:
        apt install python3-pip python3-venv python3-opencv git
    (use the distro opencv - building opencv-python on armhf takes hours)
[ ] Create a venv and install the chosen inference runtime.
[ ] Confirm the runtime imports and runs a dummy inference.
[ ] Copy the project to the Pi:
        scp -r orange_pi/ <user>@<pi-ip>:~/robosort/
    (The SD card is a single ext4 partition - Windows cannot write to it,
     so the copy has to happen after the Pi is booted, over the network.)
[ ] Run the smoke test:  python3 hello_pi.py
[ ] Confirm config.py auto-selects the "pi" profile (Linux -> pi).


================================================================
PHASE 3 - THE MODEL  (currently the biggest gap)
================================================================
[!] There is NO trained model. runs/train/robosort_yolo11/weights/ is
    EMPTY - training was started (args.yaml and train_batch*.jpg exist)
    but never finished, so no best.pt was ever produced.
    Right now classify.py is falling back to the COCO map on an
    untrained yolo11n.pt. That is not a working trash classifier.

[ ] Re-run training on the laptop to completion:  python train.py
    9 classes: Polystyrene, bread, cardboard, eggs, metal, paper,
               peels, plastic, walnuts
[ ] Record final mAP / accuracy. Decide if it is good enough to ship.
[ ] Confirm the 9-class -> BIO/NONBIO mapping in classify.py is correct.
[ ] Export the trained weights to the Pi runtime format (NCNN or ONNX).
[ ] Benchmark on the LAPTOP at the exact --infer-size you intend to ship.
    Known laptop numbers: 192x320 -> 11.0 ms, 384x640 -> 17.4 ms.
    These do NOT transfer - a desktop CPU is overhead-bound at small
    inputs; the Cortex-A7 pays roughly its pixel count.
[ ] Copy exported weights to the Pi.
[ ] Benchmark on the PI:  python3 bench.py --limit 100
[!] GO / NO-GO: if the Pi cannot hit a usable frame rate, shrink
    --infer-size, switch to a smaller model, or move to a classifier
    instead of a detector. Decide this BEFORE building the rest around it.


================================================================
PHASE 4 - CAMERA ON THE PI
================================================================
[ ] Plug in the USB webcam, confirm it enumerates:  ls /dev/video*
[ ] python3 test_cv.py --list-cameras
[ ] Set the camera index in config.py (or ROBOSORT_CAMERA).
[ ] Confirm MJPG is being negotiated - raw YUYV caps many USB cams at 5 FPS.
[ ] Vision-only run (never opens a serial port):
        python3 test_cv.py --frames 40
[ ] Score a known class folder to sanity-check accuracy on real Pi frames.


================================================================
PHASE 5 - ARDUINO MEGA FIRMWARE  (does not exist yet)
================================================================
[!] There is NO Arduino code anywhere in this repo - no .ino, no sketch
    folder. The entire hardware-control half of RoboSort is unwritten.
    Everything below is from scratch.

[ ] Create an arduino/ folder in the repo for the sketch.
[ ] Drive: 4 DC motors + motor driver. Forward/back/turn/stop.
[ ] Ultrasonics: 4 sensors, non-blocking reads.
[ ] Obstacle avoidance that can HALT movement on its own, without
    waiting on the Orange Pi (hard requirement in CLAUDE.md).
[ ] Arm servos: lift + stretch pickup sequence.
[ ] Conveyor DC motor.
[ ] Sorting servo: BIO position / NONBIO position.
[ ] Serial receiver for the B / N / X protocol from the Pi.
[ ] Decide whether the protocol needs checksums or ACK/NAK - required if
    a dropped message can cause unsafe motion.
[ ] Bench-test each subsystem alone before wiring them together.


================================================================
PHASE 6 - INTEGRATION
================================================================
[ ] Test serial from the LAPTOP to the Mega over USB first.
    The Mega does not care which host is talking to it, and this is far
    easier to debug than doing it on the Pi.
[ ] Find the Pi's serial device:  /dev/ttyS* or /dev/ttyUSB*
[ ] Wire Pi <-> Mega. Confirm voltage levels are safe (3.3V vs 5V).
[ ] Set ROBOSORT_SERIAL_PORT in the pi profile.
[ ] Dry run, no Arduino:      python3 vision.py --dry-run
[ ] Live run with the Mega:   python3 vision.py
[ ] Full loop test: detect -> classify -> send B/N -> servo sorts correctly.
[ ] Set the vision pipeline to start on boot (systemd unit).
[ ] Full rover test: navigate, avoid, pick up, convey, sort, resume.


================================================================
NOTES / GOTCHAS
================================================================
- This file is NOT tracked by git. .gitignore line 4 is "*.txt", which
  matches todo.txt. Rename it todo.md or add "!todo.txt" to .gitignore
  if you want it in version control.
- The SD card's only partition is ext4. Windows cannot read or write it,
  so nothing can be staged onto the card from the laptop.
- Armbian default login is root / 1234 on first boot only.
- Windows renumbers webcams between runs - never memorise a camera index,
  re-run --list-cameras.
- Capture is pinned at 640x480 on purpose: exactly 2x the 320 training
  size, universally supported, and it survives the port to the Pi.
