"""Open the generated scene in the installed Isaac Sim and play its USD timeline."""
import argparse
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--camera', choices=['Overview', 'PickStation', 'Inspection', 'Inspection2', 'GantryLoading', 'Separator', 'Palletising'], default='Overview')
parser.add_argument('--start', type=float, default=0, help='Start time in seconds')
parser.add_argument('--capture', help='Save an RTX viewport image after the requested frames')
parser.add_argument('--loop', action='store_true', help='Reset the complete scene after each demonstration')
parser.add_argument('--capture-suite', help='Save kiln-car lift, reject removal, separator push, pallet lift and shipping RTX views')
parser.add_argument('--headless', action='store_true')
parser.add_argument('--frames', type=int, default=0, help='Stop after N updates; 0 runs until closed')
args = parser.parse_args()
scene = Path(__file__).resolve().parent / 'output/acme_factory.usdc'
if not scene.exists():
    raise SystemExit('Run build_factory.py first.')

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless, 'width': 1600, 'height': 1000,
                     'hide_ui': False, 'disable_viewport_updates': False,
                     'open_usd': str(scene), 'renderer': 'RaytracedLighting'})
try:
    import omni.timeline
    import omni.usd
    from omni.kit.viewport.utility import get_active_viewport
    for _ in range(20):
        app.update()
    viewport = get_active_viewport()
    if viewport:
        viewport.set_active_camera('/World/Cameras/'+args.camera)
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_end_time(omni.usd.get_context().get_stage().GetEndTimeCode()/30)
    timeline.set_current_time(args.start)
    timeline.set_looping(args.loop)
    timeline.play()
    count = 0
    while app.is_running() and (args.frames == 0 or count < args.frames):
        app.update()
        count += 1
    print(f'Factory playback: {count} updates; timeline at {timeline.get_current_time():.3f} seconds', flush=True)
    if args.capture:
        import omni.replicator.core as rep
        from PIL import Image
        timeline.pause()
        product = rep.create.render_product('/World/Cameras/'+args.camera, (1600,1000))
        rgb = rep.AnnotatorRegistry.get_annotator('rgb')
        rgb.attach(product)
        rep.orchestrator.step(rt_subframes=8, delta_time=0.0)
        data = rgb.get_data()
        if data is None or not data.size:
            raise RuntimeError('RTX capture returned no image')
        Image.fromarray(data).save(Path(args.capture).resolve())
        rgb.detach(product)
        product.destroy()
        print(f'Saved RTX capture: {args.capture}', flush=True)
    if args.capture_suite:
        import omni.replicator.core as rep
        from PIL import Image
        directory = Path(args.capture_suite).resolve()
        directory.mkdir(parents=True,exist_ok=True)
        timeline.pause()
        import json
        review = json.loads((scene.parent/'flow_report.json').read_text())['review_seconds']
        for seconds,camera,name in [(review['gantry_layer_lift'],'GantryLoading','gantry_layer_lift.png'),
                                    (review['parallel_robots'],'PickStation','parallel_robots.png'),
                                    (review['separator_push'],'Separator','separator_push.png'),
                                    (review['pallet_layer_lift'],'Palletising','pallet_layer_lift.png'),
                                    (review['pallet_to_shipping'],'Palletising','pallet_to_shipping.png')]:
            timeline.set_current_time(seconds)
            for _ in range(5):
                app.update()
            product = rep.create.render_product('/World/Cameras/'+camera,(1600,1000))
            rgb = rep.AnnotatorRegistry.get_annotator('rgb')
            rgb.attach(product)
            rep.orchestrator.step(rt_subframes=8,delta_time=0.0)
            data = rgb.get_data()
            if data is None or not data.size:
                raise RuntimeError('RTX capture returned no image')
            Image.fromarray(data).save(directory/name)
            rgb.detach(product)
            product.destroy()
            print(f'Captured {name} at {seconds:.1f}s',flush=True)
finally:
    app.close()
