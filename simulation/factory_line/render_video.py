"""Render a 30-second Isaac Sim RTX walkthrough of the whole line: one shot per process step, captioned.

  render_video.py --test        one still per shot (check the framing)
  render_video.py               all frames, then H.264 MP4 at output/acme_line_30s.mp4
"""
import argparse
import json
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FPS, W, H = 30, 1920, 1080
FFMPEG = shutil.which('ffmpeg') or next(Path.home().joinpath('.cache/uv').rglob('ffmpeg-linux-x86_64-*'), None)   # system ffmpeg, else the uv one

# (video seconds, sim start, sim end, eye start, eye end, target start, target end, title, subtitle)
SHOTS = [
    (3.0, 0, 8, (27, -21, 23), (23, -16, 17), (8, 4.5, 0), (8.5, 4.8, 0),
     'ACME dehacking line', 'Kiln cart to pallet, one full stack (800 bricks) of the 28,800-brick cart'),
    (4.0, 1, 25, (-1.2, 10.4, 5.8), (-0.4, 9.6, 5.0), (2.4, 4.6, 1.0), (2.4, 4.8, 1.0),
     '1  Dehacking', 'Crane G1 lifts a full 40-brick layer off the kiln cart onto the layer belt'),
    (4.5, 33, 55, (9.3, 0.6, 4.8), (9.3, 1.6, 4.0), (9.3, 5.8, .85), (9.3, 5.8, .85),
     '2  Shade inspection + reject removal', 'Overhead camera at each station grades every brick; four FR3s lift the rejects'),
    (2.5, 55, 66, (20.0, 0.2, 3.4), (19.0, 0.8, 3.0), (15.6, 3.8, .7), (15.6, 3.6, .7),
     '3  Scrap to the crusher', 'Rejects ride the north and south scrap belts into the crusher'),
    (5.0, 64.8, 71.6, (14.2, 3.0, 3.2), (13.7, 3.5, 2.9), (12.1, 5.8, .8), (12.1, 5.8, .8),
     '4  Separator SP1 splits the pair', 'Paddle pushes layer 1 north to lane A, then layer 2 south to lane B'),
    (3.0, 108, 126, (16.8, 2.8, 3.8), (16.2, 3.2, 3.4), (13.8, 5.8, .85), (13.8, 5.8, .85),
     '5  Layer completion', 'Gantries C1 (lane A) and C2 (lane B) fill reject holes with good bricks'),
    (4.5, 126, 440, (20.5, 1.4, 6.4), (19.6, 2.0, 5.8), (13.6, 6.6, 1.5), (13.4, 6.8, 1.4),
     '6  Palletising', 'Crane G2 stacks complete layers: 10 x 4 x 14 = 560 bricks per pallet'),
    (2.0, 440.5, 452, (19.5, 1.8, 4.6), (20.5, 2.6, 4.2), (15.0, 7.0, .8), (16.0, 7.4, .8),
     '7  Full pallet to shipping', 'The full pallet rolls out; the dispenser feeds the next one in'),
    (1.5, 452, 513, (23, -16, 17), (26, -20, 22), (8.5, 4.8, 0), (8, 4.5, 0),
     'Measured in the sim', '6,366 bricks/h in  ·  6,361 good bricks/h palletised  ·  10 % reject rate assumed'),
]


def lerp(a, b, u):
    return tuple(x+(y-x)*u for x, y in zip(a, b))


def ease(u):
    return u*u*(3-2*u)


def frames():
    """Yield (frame, shot index, sim seconds, eye, target, shot progress)."""
    n = 0
    for k, (dur, s0, s1, e0, e1, t0, t1, *_ ) in enumerate(SHOTS):
        count = round(dur*FPS)
        for i in range(count):
            u = i/(count-1)
            yield n, k, s0+(s1-s0)*u, lerp(e0, e1, ease(u)), lerp(t0, t1, ease(u)), u
            n += 1


def counters(report):
    ledger, handling = report['ledger'], report['handling']
    g1 = sorted(h['release_seconds'] for h in handling if h['machine'] == 'G1')
    g2 = sorted(h['release_seconds'] for h in handling if h['machine'] == 'G2')
    picks = sorted(t['attach_seconds'] for t in report['manipulation']['tasks'])
    crushed = sorted(e['consumed_seconds'] for e in ledger if 'consumed_seconds' in e)
    ship = [p['shipped_seconds'] for p in report['pallets'] if p['shipped_seconds']]

    def at(t):
        return [('Layers dehacked', f"{sum(x <= t for x in g1)} / 20"),
                ('Rejects removed', str(sum(x <= t for x in picks))),
                ('Rejects crushed', str(sum(x <= t for x in crushed))),
                ('Bricks palletised', str(40*sum(x <= t for x in g2))),
                ('Pallets shipped', str(sum(x <= t for x in ship)))]
    return at


def overlay(path, shot, sim_t, u, at):
    from PIL import Image, ImageDraw, ImageFont
    dur, s0, s1, *_, title, sub = SHOTS[shot]
    img = Image.open(path).convert('RGBA')
    layer = Image.new('RGBA', img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    bold = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 44)
    reg = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 26)
    small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 22)
    mono = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf', 26)
    fade = min(1, u*dur/.35, (1-u)*dur/.25+.35) if shot not in (0,) else min(1, (1-u)*dur/.25+.35)
    a = int(255*max(0, fade))
    d.rounded_rectangle((36, 32, 60+max(d.textlength(title, bold), d.textlength(sub, reg)), 156), 14, fill=(10, 16, 24, int(.72*a)))
    d.rectangle((36, 32, 44, 156), fill=(245, 158, 11, a))
    d.text((62, 44), title, font=bold, fill=(255, 255, 255, a))
    d.text((62, 106), sub, font=reg, fill=(220, 228, 236, a))
    speed = (s1-s0)/dur
    clock = f"sim {sim_t:6.1f} s   x{speed:.0f}" if speed >= 1.5 else f"sim {sim_t:6.1f} s   x{speed:.1f}"
    tw = d.textlength(clock, mono)
    d.rounded_rectangle((W-tw-72, 32, W-36, 84), 12, fill=(10, 16, 24, 190))
    d.text((W-tw-54, 44), clock, font=mono, fill=(255, 255, 255, 255))
    items = at(sim_t)
    x = 36
    for label, value in items:
        w = max(d.textlength(label, small), d.textlength(value, mono))+40
        d.rounded_rectangle((x, H-110, x+w, H-30), 12, fill=(10, 16, 24, 190))
        d.text((x+20, H-102), label, font=small, fill=(180, 192, 204, 255))
        d.text((x+20, H-72), value, font=mono, fill=(255, 255, 255, 255))
        x += w+14
    # progress through the whole run
    d.rectangle((0, H-8, W, H), fill=(10, 16, 24, 200))
    d.rectangle((0, H-8, int(W*sim_t/513), H), fill=(245, 158, 11, 255))
    Image.alpha_composite(img, layer).convert('RGB').save(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--frames-dir', default=str(ROOT/'output/video_frames'))
    parser.add_argument('--subframes', type=int, default=6)
    args = parser.parse_args()
    out = Path(args.frames_dir)
    out.mkdir(parents=True, exist_ok=True)
    report = json.loads((ROOT/'output/flow_report.json').read_text())
    at = counters(report)
    plan = list(frames())
    if args.test:
        mids = {}
        for item in plan:
            if item[5] >= .5 and item[1] not in mids:
                mids[item[1]] = item
        plan = list(mids.values())

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': W, 'height': H, 'renderer': 'RaytracedLighting',
                         'open_usd': str(ROOT/'output/acme_factory.usdc')})
    try:
        import omni.timeline
        import omni.usd
        import omni.replicator.core as rep
        from PIL import Image
        from pxr import Gf, UsdGeom
        for _ in range(20):
            app.update()
        stage = omni.usd.get_context().get_stage()
        cam = UsdGeom.Camera.Define(stage, '/World/Cameras/Video')
        cam.CreateFocalLengthAttr(22)
        cam.CreateHorizontalApertureAttr(20.955)
        cam.CreateVerticalApertureAttr(20.955*H/W)
        cam.CreateClippingRangeAttr(Gf.Vec2f(.02, 500))
        xform = cam.AddTransformOp()
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_end_time(stage.GetEndTimeCode()/FPS)
        timeline.pause()
        product = rep.create.render_product('/World/Cameras/Video', (W, H))
        rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb.attach(product)
        for n, shot, sim_t, eye, target, u in plan:
            xform.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse())
            timeline.set_current_time(sim_t)
            app.update()
            rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0)
            data = rgb.get_data()
            if data is None or not data.size:
                raise RuntimeError(f'No image for frame {n}')
            name = out/(f'shot{shot}.png' if args.test else f'f{n:04d}.png')
            Image.fromarray(data[..., :3]).save(name)
            overlay(name, shot, sim_t, u, at)
            if n % 30 == 0 or args.test:
                print(f'frame {n}/{len(plan)} shot {shot} sim {sim_t:.1f}s', flush=True)
        rgb.detach(product)
        product.destroy()
        # Encode before app.close(): Isaac Sim ends the Python process on close.
        if not args.test:
            video = ROOT/'output/acme_line_30s.mp4'
            subprocess.run([str(FFMPEG), '-y', '-framerate', str(FPS), '-i', str(out/'f%04d.png'), '-c:v', 'libx264',
                            '-pix_fmt', 'yuv420p', '-crf', '18', '-preset', 'slow', '-movflags', '+faststart', str(video)], check=True)
            print(f'Saved {video}', flush=True)
    finally:
        app.close()


if __name__ == '__main__':
    main()
