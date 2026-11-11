"""Toolbox-style chromaticity features, shared or per-recording model.

This is a separate classifier from the experimental raw-NIR threshold. RGB is
derived; unlabelled cluster agreement cannot establish material accuracy.
"""
import colorsys
import hashlib
import json
import numpy as np
from skimage.color import lab2rgb,rgb2lab
from sklearn.cluster import KMeans
from captures.leafwood import normalise_channel


def toolbox_features(signal,near_ir):
    """Port readOS0/sbfn_ousterColors scheme #13, including gamut clipping.

    Keep the same fixed S/N scaling, nonlinear NIR hue mapping, and SignalPhoton
    lightness. Cluster displayed RGB's Lab a*/b*, exactly the feature definition
    in Toolbox. MATLAB/skimage colour conversion and k-means seeds can differ.
    """
    s=normalise_channel(signal);n=normalise_channel(near_ir)
    t=np.sqrt(np.clip((n-.15)/(.65-.15),0,1))
    lab=np.c_[12+88*s**.65,13*(1-t)-38*t,24*(1-t)+20*t]
    rgb=np.clip(lab2rgb(lab),0,1)
    return rgb2lab(rgb)[:,1:3]


def fit_lab_model(samples,maximum_per_scan=4096,scope='shared collection'):
    features=[];nir=[];scans=0
    for signal,near_ir in samples:
        signal=np.asarray(signal);near_ir=np.asarray(near_ir)
        if signal.shape!=near_ir.shape:raise ValueError('Signal/NIR shapes differ')
        if not len(signal):continue
        ix=np.linspace(0,len(signal)-1,min(len(signal),maximum_per_scan),dtype=int)
        features.append(toolbox_features(signal[ix],near_ir[ix]));nir.append(near_ir[ix]);scans+=1
    if not features:raise ValueError('No valid paired channel samples')
    x=np.concatenate(features);n=np.concatenate(nir)
    if np.max(np.ptp(x,axis=0))<1e-5:raise ValueError('No distinct Lab classes')
    fit=KMeans(n_clusters=2,n_init=3,random_state=0,max_iter=100).fit(x)
    mean_nir=[float(np.mean(n[fit.labels_==i])) for i in range(2)]
    leaf=int(np.argmax(mean_nir))
    model=dict(version=1,method='Toolbox scheme-13 displayed Lab a*/b* k-means; higher mean raw NIR cluster provisionally leaf',
               scope=scope,lab_centres=fit.cluster_centers_.tolist(),leaf_cluster=leaf,mean_raw_nir_by_cluster=mean_nir,
               scans=scans,training_samples=len(x),maximum_samples_per_scan=maximum_per_scan,
               source='GreenSprayToolbox utility/readOS0.m sbfn_ousterColors',
               note='Feature-equivalent port, not bit-identical MATLAB output; no botanically labelled validation')
    model['model_sha256']=hashlib.sha256(json.dumps(model,sort_keys=True).encode()).hexdigest()
    return model


def classify_lab(signal,near_ir,model):
    x=toolbox_features(signal,near_ir);centres=np.asarray(model['lab_centres'])
    distances=np.sum((x[:,None,:]-centres[None,:,:])**2,axis=2)
    leaf_index=model['leaf_cluster'];leaf=distances[:,leaf_index]<distances[:,1-leaf_index]
    wood=distances[:,leaf_index]/np.maximum(distances.sum(axis=1),1e-12)
    confidence=np.abs(2*wood-1)
    palette=[]
    for rgb in [(151,98,53),(48,183,70)]:
        h,s,_=colorsys.rgb_to_hsv(*(np.asarray(rgb)/255));palette.append(colorsys.hsv_to_rgb(h,s,1.))
    brightness=normalise_channel(signal)**.65
    rgb=np.floor(255*np.asarray(palette)[leaf.astype(int)]*brightness[:,None]+.5).astype('u1')
    return dict(leafwood_class=np.where(leaf,2,1).astype('u1'),wood_score=wood.astype('f4'),
                leafwood_confidence=confidence.astype('f4'),rgb=rgb)
