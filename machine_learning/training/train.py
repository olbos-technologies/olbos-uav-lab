"""Synthetic-only experiment. Not evidence of real fault detection.
Split by complete flight episodes before fitting; unseen test seed and wind regime.
No pickle/joblib from untrusted sources should ever be loaded.
"""
from pathlib import Path
import json,time,platform
import numpy as np
import sklearn,joblib
from sklearn.ensemble import IsolationForest,RandomForestClassifier,RandomForestRegressor
from sklearn.metrics import precision_recall_fscore_support,confusion_matrix,mean_absolute_error
OUT=Path(__file__).resolve().parents[2]/'artifacts';OUT.mkdir(exist_ok=True)
FEATURES=['rpm','voltage','current','accel','gyro','vibration','position_error','temperature']
def generate(seed,episodes,wind):
 rng=np.random.default_rng(seed);features=[];labels=[]
 for episode in range(episodes):
  bias=rng.normal(0,.2);fault=episode%4
  for tick in range(100):
   x=np.array([5000,16,8,9.81,.03,.1,.2,30])+rng.normal(0,[80,.1,.4,.1,.01,.02,.05,1])
   x[4:7]+=np.abs(rng.normal(0,wind,3));x[2]+=bias
   y=0 if tick<40 else fault
   if y==1:x[1]-=2;x[2]+=4
   if y==2:x[5]+=.4;x[0]-=500
   if y==3:x[6]+=2;x[4]+=.1
   features.append(x);labels.append(y)
 return np.array(features,dtype=np.float32),np.array(labels)
def metrics(y,p):
 binary=y>0; predicted=p>0
 precision,recall,f1,_=precision_recall_fscore_support(binary,predicted,average='binary',zero_division=0)
 tn,fp,fn,tp=confusion_matrix(binary,predicted,labels=[False,True]).ravel()
 return dict(precision=float(precision),recall=float(recall),f1=float(f1),false_positive_rate=float(fp/max(1,fp+tn)),confusion_matrix=[[int(tn),int(fp)],[int(fn),int(tp)]])
def routes(seed,n):
 rng=np.random.default_rng(seed);x=rng.uniform([20,0,0,5],[500,30,12,40],(n,4))
 # artificial target deliberately nonlinear; units energy Wh, duration seconds
 y=.02*x[:,0]+.09*x[:,1]+.006*x[:,0]*x[:,2]+rng.normal(0,.2,n)
 return x,y
train_x,train_y=generate(41,40,.015);test_x,test_y=generate(93,20,.04)
iso=IsolationForest(n_estimators=100,random_state=42,contamination=.03).fit(train_x[train_y==0])
rf=RandomForestClassifier(n_estimators=100,max_depth=8,random_state=42,n_jobs=1).fit(train_x,train_y)
start=time.perf_counter();pred=rf.predict(test_x);elapsed=time.perf_counter()-start
joblib.dump(rf,OUT/'health_rf.joblib');joblib.dump(iso,OUT/'health_isolation.joblib')
rx,ry=routes(11,2000);tx,ty=routes(12,500)
route=RandomForestRegressor(n_estimators=80,max_depth=12,random_state=42,n_jobs=1).fit(rx,ry)
joblib.dump(route,OUT/'route_rf.joblib')
report={'scope':'SYNTHETIC_ONLY, advisory outputs, no flight integration','features':FEATURES,
 'versions':{'python':platform.python_version(),'numpy':np.__version__,'sklearn':sklearn.__version__},
 'train_samples':len(train_y),'test_samples':len(test_y),'split':'disjoint complete episodes; seeds 41 / 93; test wind shifted',
 'random_forest':metrics(test_y,pred),'isolation_forest':metrics(test_y,iso.predict(test_x)==-1),
 'batch_inference_ms_per_sample':elapsed*1000/len(test_y),'batch_latency_warning':'batch average; not real-time p95/p99',
 'route_energy_mae_wh':float(mean_absolute_error(ty,route.predict(tx))),
 'route_deterministic_mae_wh':float(mean_absolute_error(ty,.02*tx[:,0]+.09*tx[:,1]+.006*tx[:,0]*tx[:,2])),
 'limitations':['faults are artificial shifts','no calibration or physical validation','no ONNX export/runtime validation','route regression must rank already safe candidates only']}
(OUT/'ml_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
