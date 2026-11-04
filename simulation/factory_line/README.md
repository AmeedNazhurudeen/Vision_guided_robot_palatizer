# ACME factory — dehacking, reject removal, palletising

One complete kiln-car stack (10 x 4 x 20 = 800 bricks) runs through the line as fast as the machines allow. Measured in the sim: 6,366 bricks/h in, 6,361 good bricks/h palletised (brief: 1,500-3,000). One full 560-brick pallet ships at 445 s; the run ends at 513 s.

```bash
~/env_isaacsim/bin/python simulation/factory_line/run_factory.py            # Overview camera from 0 s
~/env_isaacsim/bin/python simulation/factory_line/run_factory.py --camera PickStation --start 80
~/env_isaacsim/bin/python simulation/factory_line/run_factory.py --camera Separator --start 105   # SP1 splitting a layer pair
```

## Flow

1. **G1 crane** lifts layers off stack (1,5) of the kiln car, top layer first, onto the layer belt LB.
2. **Layer pairs:** two layers index together (80 bricks on the belt). The lead layer stops at FR3 station 2, the trailing layer at station 1, so both robot pairs work at the same time.
3. **Overhead camera at each station** (directly above the pick area, 1.4 m above the bricks) images the layer: shade, defects and brick pose.
4. **FR3 reject removal:** a Franka on each side of each station (R1-R4). The north robot takes rows 5-9, the south robot rows 0-4 (one side cannot reach the far rows). Each lifts the bad bricks onto its **scrap belt**, which runs beside the line to the **crusher**.
5. **Split (separator SP1):** after the FR3 stations each layer runs onto the split table at the end of LB. An overhead two-way pusher lowers its paddle behind the layer and pushes it sideways 0.4 m: the lead layer north onto fill lane A, the trailing layer south onto fill lane B. The paddle then lifts clear (it travels raised over the bricks) and the layer conveys onto its lane. Push 1.0 s plus 0.3 s lower and 0.3 s lift; SP1 is 8 % busy.
6. **Two cartesian gantries** (C1 on lane A, C2 on lane B) fill the holes with good bricks from their lane's donor layer, so every layer leaves complete (40 bricks).
7. **G2 crane** lifts whichever lane's layer is complete first onto the pallet: 10 x 4 x 14 = 560 bricks. A full pallet rolls to shipping.

## Production rate

- Mean layer-pair cycle: 45.2 s (worst 65.2 s) = 6,366 bricks/h. Bottleneck: G1 (69 % busy), then G2 (58 %). The FR3s are 28-49 % busy, the gantries about 20 % and SP1 8 %.
- The previous one-layer-at-a-time line did 2,986 bricks/h in and about 2,650 good bricks/h palletised.
- FR3 reject cycle: 8.0 s, peak joint speed 0.90 of the FR3 limits. Reject rate 10 % is assumed (the brief gives none).

## Limits

Shade/defect classification is synthetic metadata, not image inference from the inspection camera. SP1 moves each layer as a rigid block: in a real push, bricks behind a reject hole would close up toward the paddle side. Motion is recorded kinematics (MuJoCo IK for the FR3s), not PhysX contact dynamics. Pallets are mixed-shade; single-shade pallets would need one donor/fill line per shade.

## Review images

The launcher can produce actual Isaac Sim RTX images of loading, parallel manipulation and pallet lifting:

```bash
~/env_isaacsim/bin/python simulation/factory_line/run_factory.py --headless --frames 120 --capture-suite simulation/factory_line/output/review
```

Images: `output/review/gantry_layer_lift.png`, `parallel_robots.png`, `separator_push.png`, `pallet_layer_lift.png`, `pallet_to_shipping.png`.

## Checks

```bash
cd simulation/factory_line
~/env_isaacsim/bin/python build_factory.py
~/env_isaacsim/bin/python validate_factory.py
~/env_isaacsim/bin/python audit_collisions.py
```

`validate_factory.py` audits all 800 moving bricks through the complete 511-second run at 30 Hz: surface support, G1/G2/C1 and robot attachment, every bad brick crushed and every pallet layer complete, north/south robots never inside the layer together, the SP1 paddle in contact with every layer it pushes (lead to A, trailing to B) and never cutting into a brick, FR3 joint position and velocity limits, brick overlaps, bricks against fixed equipment and kiln stacks, G1's frame against the neighbouring stacks, and no machine standing in another (fixed pairs plus moving crane, gantry and pallet parts every 0.2 s). `output/validation_report.json` records the results.

`audit_collisions.py` uses the FR3 collision meshes, vacuum-tool cylinders, equipment boxes and actual moving brick occupancy to check every robot cycle at 30 Hz. It includes all four arms together, and proves the cranes, gantries, SP1 and pallets stay outside FR3 reach (nearest: the SP1 paddle, 0.12 m). The intentional bolted base mounting contact is excluded. Results are in `output/collision_report.json`.

These are sampled geometric checks of recorded kinematic motion. They are not contact-driven PhysX dynamics, a suction-force calculation, a certified safety assessment or a measured production throughput result. Robot links use MuJoCo FK and IK; the exported USD plays in Isaac Sim without MuJoCo at runtime. Shade classification is synthetic metadata, not image inference. The mixed output contains several shades but is not a production 1:1:1 recipe controller.

## References and dimensions

The starting references were `UTUM Brick Coloration.pdf` and the supplied `ACME sim` v5 layout. The scene preserves 245 × 67 × 69 mm bricks and the 28,800-brick kiln-car count. There are 28,000 static bricks and 800 moving bricks (one full stack). The v5 4 × 10 layer arrangement and assumed 6 × 6 stack arrangement are retained. The PDF describes the 40-brick layer as 10 × 4 instead; the footprint here follows v5.

The layout is revised: the v5 shade lanes, blend belt, four formers and pallet stations are replaced by two scrap belts to the crusher, the SP1 separator, the C1/C2 layer-completion gantries and one G2 pallet line. The kiln-car deck rides on its rails, and the control cabinet sits north of the line. `assets/components_v5.csv` is the original reference; `factory_process.revised_rows()` applies the new layout.

Rebuilding the FR3 visuals reads the local Menagerie XML and meshes at `assets/franka_fr3/`. `assets/FR3_LICENSE` retains their license. The exported `output/acme_factory.usdc` contains its geometry and requires no remote asset downloads. Builds replace the scene only after generation succeeds.
