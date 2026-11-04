"""Build labeled H.264 videos and an index from completed scene-suite captures."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def render_video(capture, target, title, report):
    if target.exists():
        return
    controls=[json.loads(p.read_text()) for p in sorted(capture.glob('control_*.json'))]
    meta=json.loads((capture/'000_verifai/overhead.json').read_text())
    options=json.loads((capture/'experiment.json').read_text())['options']
    inv=np.linalg.inv(np.asarray(meta['camera_world_matrix']))
    focal=meta['width']/(2*math.tan(math.radians(meta['fov_degrees'])/2))
    def pixel(point):
        q=inv@np.r_[point,1.]
        if q[0]<=0:return None
        return tuple(int(round(v)) for v in (meta['width']/2+focal*q[1]/q[0],meta['height']/2-focal*q[2]/q[0]))
    source=cv2.VideoCapture(str(capture/'000_verifai/overhead.mp4'))
    temp=target.with_name(target.stem+'-tmp.mp4')
    writer=cv2.VideoWriter(str(temp),cv2.VideoWriter_fourcc(*'mp4v'),10,(1280,1080))
    if not writer.isOpened():raise RuntimeError('Could not open annotated video')
    outcome='PASS' if report['passed'] else 'INCOMPLETE / FAIL'
    samples=[];index=0
    while True:
        ok,frame=source.read()
        if not ok:break
        canvas=np.zeros((1080,1280,3),dtype=np.uint8);canvas[:960]=frame
        cv2.putText(canvas,f'{title} | {outcome}',(20,989),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1,cv2.LINE_AA)
        cv2.putText(canvas,'Cyan: actual motion | Yellow: native prediction | Magenta: native target | Green: projected target',
                    (20,1019),cv2.FONT_HERSHEY_SIMPLEX,.54,(230,230,230),1,cv2.LINE_AA)
        mission=options.get('mission',{})
        if mission.get('kind')=='red_light_stop':
            line=mission['stop_line'];yaw=math.radians(line['yaw'])
            a=pixel([line['x']-1.75*math.sin(yaw),line['y']+1.75*math.cos(yaw),line['z']+.1])
            b=pixel([line['x']+1.75*math.sin(yaw),line['y']-1.75*math.cos(yaw),line['z']+.1])
            if a and b:cv2.line(canvas,a,b,(0,0,255),3,cv2.LINE_AA)
        if index and index-1<len(controls):
            record=controls[index-1];d=record['debug']
            a=pixel(d['native_target_xyz']);b=pixel(d['projected_target_xyz'] or d['native_target_xyz'])
            if a and b:
                cv2.line(canvas,a,b,(255,0,255),2,cv2.LINE_AA)
                cv2.circle(canvas,a,8,(255,0,255),2,cv2.LINE_AA);cv2.circle(canvas,b,5,(0,255,0),-1,cv2.LINE_AA)
            text=f"Target shift {d['projection_distance_m']:.2f} m | speed target {d['target_speed_kmh']/3.6:.2f} m/s | age at control {record['elapsed_sec']:.1f}s"
            if mission:text+=' | Red line: stopping boundary'
            if options.get('hold_visual_stop'):
                text+=' | stop hold: '+('ON' if d['visual_stop_hold']['active'] else 'off')
            cv2.putText(canvas,text,(20,1050),cv2.FONT_HERSHEY_SIMPLEX,.54,(210,230,255),1,cv2.LINE_AA)
        writer.write(canvas)
        if index%30==0:samples.append(cv2.resize(canvas,(384,324)))
        index+=1
    source.release();writer.release()
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-i',str(temp),'-c:v','libx264','-preset','fast',
                    '-crf','20','-pix_fmt','yuv420p','-movflags','+faststart','-an',str(target)],check=True)
    temp.unlink()
    if samples:cv2.imwrite(str(target.with_suffix('.jpg')),np.hstack(samples))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path)
    args=parser.parse_args();out=args.output.resolve();status=json.loads((out/'status.json').read_text())
    archived_manifest=out/'manifest.json'
    manifest=json.loads((archived_manifest if archived_manifest.exists() else Path(status['manifest'])).read_text())
    cases={c['name']+'-route':c for c in manifest['scenarios']}
    results=[];fig,axes=plt.subplots(5,2,figsize=(13,15),constrained_layout=True)
    for item in status['results']:
        name=item['name'];case=cases[name];capture=out/name
        if not (capture/'projection-audit.json').exists():continue
        report=json.loads((capture/'projection-audit.json').read_text())
        video=out/(name+'-targets.mp4')
        render_video(capture,video,case.get('title',name),report)
        rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader((capture/'000_verifai/ego.csv').open())]
        body=json.loads((capture/'footprint-audit.json').read_text())['frames']
        axis=axes.flat[int(name[:2])-1]
        axis.plot([r['time_s'] for r in rows],[r['speed'] for r in rows],label='Speed (m/s)')
        axis.plot([r['time_s'] for r in body],[-r['max_corner_overhang_m'] for r in body],label='Corner clearance (m)')
        axis.axhline(0,color='grey',lw=.5);axis.set_title(case.get('title',name)+' — '+('PASS' if report['passed'] else 'INCOMPLETE / FAIL'))
        axis.set_xlabel('Simulation time (s)');axis.grid(alpha=.2);axis.legend(fontsize=8)
        results.append(dict(name=name,title=case.get('title',name),video=str(video),**{k:v for k,v in report.items() if k not in ('projection_trace','near_stop_times_s','body_departure_times_s')}))
    fig.savefig(out/'scene-metrics.png',dpi=150);fig.savefig(out/'scene-metrics.pdf');plt.close(fig)
    summary=dict(state=status['state'],expected_cases=len(manifest['scenarios']),completed=len(status['results']),
                 passed=sum(r['passed'] for r in results),results=results,scope=manifest.get('note'))
    (out/'suite-results.json').write_text(json.dumps(summary,indent=2)+'\n')
    lines=['# Ten driving scenes','',f"Completed {len(status['results'])}; passed {summary['passed']}. Every recorded attempt is retained.",'',
           '| Scenario | Outcome | End time | Speed range | Corner clearance | Video |','| --- | --- | --- | --- | --- | --- |']
    for r in sorted(results,key=lambda x:x['name']):
        lines.append(f"| {r['title']} | {'Pass' if r['passed'] else 'Incomplete / fail'} | {r['duration_s']:.1f} s | {r['min_speed_mps']:.2f}–{r['max_speed_mps']:.2f} m/s | {r['min_corner_clearance_m']:.3f} m | [Watch]({Path(r['video']).name}) |")
    lines+=['','[Measured speed and clearance](scene-metrics.png)','',manifest.get('note','')]
    (out/'VIDEOS.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='results'}))


if __name__=='__main__':main()
