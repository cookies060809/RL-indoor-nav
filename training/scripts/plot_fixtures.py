from pathlib import Path
import os
ROOT=Path(__file__).resolve().parents[2]
os.environ['MPLCONFIGDIR']=str(ROOT/'.cache/matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

names=['empty','front_wall','left_wall','corner','invalid','offset_sensor']
fig,axes=plt.subplots(2,3,figsize=(12,8))
for ax,name in zip(axes.flat,names):
    obs=np.load(ROOT/'training/tests/fixtures'/f'{name}.npz')['observation']; n=int(np.sqrt((len(obs)-8)/2))
    grid=obs[:n*n].reshape(n,n)
    ax.imshow(grid,origin='lower',extent=(-2.4,2.4,-2.4,2.4),vmin=0,vmax=1,cmap='gray_r',interpolation='nearest')
    circle=plt.Circle((0,0),.226,fill=False,color='red'); ax.add_patch(circle)
    ax.set_title(name); ax.set_xlabel('robot x forward (m)'); ax.set_ylabel('robot y left (m)')
fig.tight_layout(); fig.savefig(ROOT/'docs/reports/bev_fixtures.png',dpi=160)
print(ROOT/'docs/reports/bev_fixtures.png')
