"""Render three aligned recorded trajectories; never synthesize experimental results."""
import json
from pathlib import Path
import shutil
import numpy as np


def load_runs(paths):
    rows=[]; manifests=[]; geometry=[]
    for path in paths:
        manifests.append(json.loads((path/"run.json").read_text()))
        samples=[json.loads(line) for line in (path/"telemetry.jsonl").read_text().splitlines() if line.strip()]
        if len(samples)<2: raise ValueError(f"insufficient recorded samples: {path}")
        rows.append(samples)
        geometry.append(json.loads((path/"scenario.json").read_text()))
    keys=("scenario","seed","world_sha256","source_sha256","calibration_sha256")
    if any(any(m.get(k)!=manifests[0].get(k) for k in keys) for m in manifests):
        raise ValueError("three runs must share scenario, seed, world, source and calibration")
    if len(set(m["strategy"] for m in manifests)) != 3:
        raise ValueError("select three different strategies")
    return rows,manifests,geometry


def render(paths, output):
    if not shutil.which("ffmpeg"): raise RuntimeError("ffmpeg is required")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, FFMpegWriter
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    rows,manifests,geometry=load_runs(paths)
    output.parent.mkdir(parents=True,exist_ok=True)
    fig=plt.figure(figsize=(16,9),facecolor="#f4f6fa")
    axes=[]; lines=[]; markers=[]; labels=[]
    duration=max(r[-1]["sim_time"]-r[0]["sim_time"] for r in rows)
    for i in range(3):
        ax=fig.add_subplot(2,3,i+1,projection="3d"); axes.append(ax)
        for box in geometry[i]["boxes"]:
            cx,cy,cz=box["center"]; sx,sy,sz=np.array(box["size"])/2
            if box["name"] in ("end_wall","start_wall"): continue
            vertices=np.array([[x,y,z] for x in (-sx,sx) for y in (-sy,sy) for z in (-sz,sz)])
            angle=box.get("yaw",0); c,s=np.cos(angle),np.sin(angle)
            rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
            vertices=vertices@rotation.T+[cx,cy,cz]
            faces=[[vertices[j] for j in ids] for ids in ((0,1,3,2),(4,5,7,6),(0,1,5,4),(2,3,7,6),(0,2,6,4),(1,3,7,5))]
            ax.add_collection3d(Poly3DCollection(faces,alpha=.15,facecolor="#778899",edgecolor="#bbc4ce",linewidth=.3))
        ax.set(xlim=(-1,14),ylim=(-2.5,2.5),zlim=(0,4),xlabel="x (m)",ylabel="y (m)",zlabel="z (m)")
        ax.set_title(manifests[i]["strategy"]+" | "+manifests[i]["status"])
        line,=ax.plot([],[],[],color="#167c80",lw=2); lines.append(line)
        marker,=ax.plot([],[],[],"o",color="#d26a35"); markers.append(marker)
        labels.append(ax.text2D(.02,.02,"",transform=ax.transAxes,fontsize=9))
        plot=fig.add_subplot(2,3,i+4)
        for field,color in (("AL","#27817b"),("PL","#d15e42"),("margin","#4357a5")):
            plot.plot([x["sim_time"]-rows[i][0]["sim_time"] for x in rows[i]],
                      [x.get(field) if x.get(field) is not None else np.nan for x in rows[i]],label=field,color=color)
        plot.axhline(0,color="gray",lw=.6); plot.set(xlabel="Task simulation time (s)",ylabel="m")
        plot.legend(frameon=False)
    title=fig.suptitle("IMPACT | SIMULATED | Recorded-state replay | Ground truth for evaluation only",fontsize=14)
    def update(frame):
        seconds=frame/20
        for i,samples in enumerate(rows):
            until=samples[0]["sim_time"]+seconds
            index=max(0,int(np.searchsorted([s["sim_time"] for s in samples],until,side="right")-1))
            index=min(index,len(samples)-1)
            points=np.array([s["truth"] for s in samples[:index+1]])
            lines[i].set_data_3d(points[:,0],points[:,1],points[:,2])
            markers[i].set_data_3d([points[-1,0]],[points[-1,1]],[points[-1,2]])
            row=samples[index]
            labels[i].set_text(f"t={seconds:.1f}s  {row.get('intent')} / {row.get('phase')}\n"
                               f"True error={row['error_3d_m']:.3f} m  clearance={row['clearance_m']:.3f} m")
        return lines+markers+labels
    fig.tight_layout(rect=(0,0,1,.95))
    animation=FuncAnimation(fig,update,frames=max(1,int(duration*20)+1),interval=50,blit=False)
    animation.save(str(output),writer=FFMpegWriter(fps=20,codec="libx264",extra_args=["-pix_fmt","yuv420p"]))
    plt.close(fig)
    print(f"Recorded-state replay saved: {output}; original Gazebo records remain in each run directory")
