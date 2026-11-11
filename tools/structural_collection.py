"""Add full-height coverage audits to QC without changing delivered geometry."""
from pathlib import Path
from tools.check_structural_alignment import check_dataset


def add_structural_checks(stage):
    stage=Path(stage);results=[]
    for folder in sorted((stage/'pointclouds').iterdir()):
        if not folder.is_dir():continue
        dataset=folder.name;destination=stage/'QC'/dataset/'structural_alignment'
        report=check_dataset(stage,dataset,destination)
        results.append(dict(dataset=dataset,coverage=report['coverage'],observable=report['observable'],
                            maximum_delivered_imu_up_tilt_deg=report['maximum_delivered_imu_up_tilt_deg']))
        index=stage/'QC'/dataset/'index.html'
        if index.exists():
            text=index.read_text();marker='<!-- full-height-structural-audit -->'
            if marker not in text:
                text+='\n'+marker+'<h2>Full-height structural coverage</h2><p>Coverage audit only: identity checks do not establish correct registration. Sparse shared same-face wood geometry leaves upper orientation unverified. No V symmetry, horizontal-bar target or vertical-trunk target is imposed.</p><p><a href="structural_alignment/structural_alignment.json">Coverage and orientation data</a></p><img style="max-width:100%" src="structural_alignment/orthographic_profile.png">\n'
                index.write_text(text)
        print('Full-height coverage audit:',dataset,report['coverage'],'observable',report['observable'],flush=True)
    import json
    (stage/'QC/structural_alignment_summary.json').write_text(json.dumps(dict(
        purpose='Coverage audit; unchanged primary outputs; not a claim of survey accuracy or orientation correctness',
        datasets=results),indent=2)+'\n')
    return results
