"""Render publication figures from recorded results; does not run experiments."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parents[1]; out=root/'results'/'figures'; out.mkdir(exist_ok=True)
paired=[json.loads(s) for s in (root/'results/paired_jetson_paper_geometry.jsonl').read_text().splitlines() if s.strip()]
policy=json.loads((root/'results/measured_uncertainty_jetson_paper_geometry.json').read_text())
feedback=json.loads((root/'results/feedback_calibration_ablation.json').read_text())['results']
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
fig,ax=plt.subplots(figsize=(6.5,4))
for label,key,color in [('Precomputed lookup','lookup','#126782'),('Online recomputation','online_recompute','#c55a11')]:
    values=np.sort([p['latency_us'][key]/1000 for p in paired]); ax.step(values,np.arange(1,len(values)+1)/len(values),where='post',label=label,color=color)
ax.set_xscale('log'); ax.set_xlabel('Computation latency (ms; logarithmic scale)'); ax.set_ylabel('Empirical cumulative fraction'); ax.legend(loc='lower right'); ax.grid(alpha=.2); ax.set_title('Jetson: 105 paired queries, identical decisions'); fig.tight_layout()
for ext in ('png','pdf'): fig.savefig(out/('paired_latency.'+ext),dpi=220)
plt.close(fig)
fig,ax=plt.subplots(figsize=(6.5,4)); x=np.arange(3)
for i,(key,label,color) in enumerate([('adaptive','Adaptive threshold','#126782'),('fixed','Fixed threshold 0.05','#c55a11')]):
    values=[s['accepted'][key]/s['cases_per_policy']*100 for s in policy['summaries']]
    bars=ax.bar(x+(i-.5)*.32,values,.32,label=label,color=color)
    for b,v in zip(bars,values): ax.text(b.get_x()+b.get_width()/2,v+1,'{:.1f}%'.format(v),ha='center',fontsize=9)
ax.set_xticks(x,labels=['Measured + allowance','4x covariance\n(synthetic)','16x covariance\n(synthetic)']); ax.set_ylim(0,110); ax.set_ylabel('Accepted model queries (%)'); ax.set_title('Matched policy comparison: 120 cases per regime'); ax.legend(loc='lower left'); fig.tight_layout()
for ext in ('png','pdf'): fig.savefig(out/('uncertainty_policy.'+ext),dpi=220)
plt.close(fig)
fig,ax=plt.subplots(figsize=(5.5,4)); values=[feedback[k]['repeat']['translation_rms_mm'] for k in ('command_cache','measured_feedback')]
bars=ax.bar(['Command-cache calibration','Measured-feedback calibration'],values,color=['#c55a11','#126782'])
for b,v in zip(bars,values): ax.text(b.get_x()+b.get_width()/2,v+.3,'{:.2f} mm'.format(v),ha='center')
ax.set_ylim(0,max(values)*1.18); ax.set_ylabel('Fixed-board repeat position RMS (mm)'); ax.set_title('Same 20 training and 20 repeat captures'); fig.tight_layout()
for ext in ('png','pdf'): fig.savefig(out/('feedback_ablation.'+ext),dpi=220)
plt.close(fig); print('Saved six figure files to',out)
