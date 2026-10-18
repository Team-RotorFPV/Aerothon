#!/usr/bin/env python3
"""Export recorded camera frames with their real capture-time spacing.

The original AVI and capture ledger remain untouched. This changes playback
timing only; it does not create camera observations or flight telemetry.
"""
import argparse
import json
import subprocess
from pathlib import Path


def export(out, name='onboard_sim_time.mp4', reuse_extracted=False):
    out=Path(out).resolve()
    if Path(name).name!=name or Path(name).suffix!='.mp4':
        raise SystemExit('Export name must be a local MP4 filename')
    target=out/name
    images=out/'exported_camera_frames'
    if target.exists() or (images.exists() and not reuse_extracted):
        raise SystemExit('Timeline export already exists')
    frames=[json.loads(line) for line in (out/'camera_frames.jsonl').open()]
    if len(frames)<2:raise SystemExit('Insufficient recorded frames')
    stamps=[r['capture_stamp'] for r in frames]
    if any(b<=a for a,b in zip(stamps,stamps[1:])):
        raise SystemExit('Capture timestamps are not strictly increasing; original recording retained')
    if not reuse_extracted:
        images.mkdir()
        subprocess.run(['ffmpeg','-v','error','-i',str(out/'onboard.avi'),
                        '-fps_mode','passthrough',str(images/'frame_%08d.png')],check=True)
    paths=sorted(images.glob('frame_*.png'))
    if len(paths)!=len(frames):raise SystemExit('Video/frame ledger count mismatch')
    ledger=target.with_suffix('.ffconcat')
    # Quantize absolute capture times once to the demuxer's millisecond clock.
    # Integer intervals avoid cumulative rounding at the demuxer's resolution.
    ticks=[round(t*1000) for t in stamps]
    if any(b<=a for a,b in zip(ticks,ticks[1:])):
        raise SystemExit('Capture intervals are below export clock resolution')
    lines=['ffconcat version 1.0']
    for i,path in enumerate(paths):
        quoted=path.as_posix().replace("'","'\\''")
        duration=(ticks[i+1]-ticks[i])/1000 if i+1<len(stamps) else .2
        lines.extend([f"file '{quoted}'",'option framerate 1000',f'duration {duration:.3f}'])
    quoted=paths[-1].as_posix().replace("'","'\\''")
    lines.extend([f"file '{quoted}'",'option framerate 1000'])
    ledger.write_text('\n'.join(lines)+'\n')
    subprocess.run(['ffmpeg','-v','error','-safe','0','-f','concat','-i',str(ledger),
                    '-fps_mode','vfr','-c:v','libx264','-bf','0','-pix_fmt','yuv420p',
                    '-movflags','+faststart',str(target)],check=True)
    subprocess.run(['ffmpeg','-v','error','-i',str(target),'-f','null','-'],check=True)
    probe=subprocess.check_output(['ffprobe','-v','error','-count_frames',
        '-select_streams','v:0','-show_entries','stream=nb_read_frames,duration,width,height',
        '-of','json',str(target)],text=True)
    measured=json.loads(probe)['streams'][0]
    expected=stamps[-1]-stamps[0]+.2
    if abs(float(measured['duration'])-expected)>.1:
        raise SystemExit('Export duration disagrees with capture timeline')
    result=dict(original_frames=len(frames),first_capture_stamp=stamps[0],
        last_capture_stamp=stamps[-1],expected_duration_s=expected,export=measured,
        playback='variable frame rate from recorded capture intervals',
        final_hold_s=.2,decode_pass=True)
    (out/'video_timeline.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('out',type=Path)
    p.add_argument('--name',default='onboard_sim_time.mp4')
    p.add_argument('--reuse-extracted',action='store_true',help='reuse verified extracted frames after a failed timing export')
    args=p.parse_args()
    export(args.out,args.name,args.reuse_extracted)
