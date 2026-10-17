"""Optional export. Not validated in initial delivery. Run only after train.py."""
from pathlib import Path
import joblib
from skl2onnx import convert_sklearn
from skl2onnx.common.data_types import FloatTensorType
root=Path(__file__).resolve().parents[2]/'artifacts'
model=joblib.load(root/'health_rf.joblib')
onnx=convert_sklearn(model,initial_types=[('telemetry',FloatTensorType([None,8]))],options={id(model):{'zipmap':False}},target_opset=17)
(root/'health_rf.onnx').write_bytes(onnx.SerializeToString())
print('Exported; numerical parity and C++ runtime integration still required.')
