from pathlib import Path
import os
ROOT=Path(__file__).resolve().parents[2]
os.environ['MPLCONFIGDIR']=str(ROOT/'.cache/matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

names=['empty','front_wall','left_wall','corner','invalid','above_max','offset_sensor']
fig,axes=plt.subplots(2,4,figsize=(14,7))
for ax in axes.flat:
    ax.set_visible(False)
for ax,name in zip(axes.flat,names):
    ax.set_visible(True)
    obs=np.load(ROOT/'training/tests/fixtures'/f'{name}.npz')['observation']
    n=int(np.sqrt((len(obs)-8)/2))
    grid=obs[:n*n].reshape(n,n)
    ax.imshow(grid,origin='lower',extent=(-2.4,2.4,-2.4,2.4),vmin=0,vmax=1,cmap='gray_r',interpolation='nearest')
    circle=plt.Circle((0,0),.226,fill=False,color='red')
    ax.add_patch(circle)
    ax.set_title(name)
    ax.set_xlabel('robot x forward (m)')
    ax.set_ylabel('robot y left (m)')
fig.tight_layout()
target=ROOT/'docs/reports/yy_v2/bev_fixtures.png'
target.parent.mkdir(parents=True,exist_ok=True)
fig.savefig(target,dpi=160)
print(target)
