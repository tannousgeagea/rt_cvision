
from core import push, client

model_name = "iserlohn.amk.want:waste.segments"
model_path = "/media/appuser/mlflow/base.segments.pt"

push(model_name=model_name, model_path=model_path)

# client.delete_registered_model(name=model_name)