# BOM (India-sourced)

[`Jero_BOM_India.xlsx`](Jero_BOM_India.xlsx) has three tabs: **BOM** (vendors, links, stock, INR incl. GST,
Ordered?/ETA columns), **Timeline** and **Option comparison**. Prices were checked on 25 Sep 2026.

- Core build (P0): about ₹54k, including 2 spare servos. All-in: about ₹64.6k.
- **Servo variant matters:** buy only **STS3215-C001 (7.4 V, 1:345)**. The sim's actuator model
  (`BAM feetech_sts3215_7_4V`) and `BEST_WALK_ONNX_2` are tuned for it. Evelta had 80 in stock.
- Not in the BOM (already in the lab): 3D printer, Jetson Orin Nano Super, GPU server.

Upstream reference BOM (EUR): see the link in the
[Open_Duck_Mini README](https://github.com/apirrone/Open_Duck_Mini#bom).
