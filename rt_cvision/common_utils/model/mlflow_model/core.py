import os
import mlflow
import logging
from ultralytics import YOLO
from mlflow.tracking import MlflowClient
from common_utils.model.mlflow_model.utils import print_model_info, print_model_version_info

# Initialize the MLflow Client
mlflow.set_tracking_uri(f"{os.getenv('MLFLOW_TRACKING_URI')}")
client = MlflowClient()

ALIAS_PROD = 'production'
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


class ModelWrapper(mlflow.pyfunc.PythonModel):
    def __init__(self, weights):
        self.weights = weights

    def load_context(self, context):
        # Load the YOLO model
        self.model = YOLO(context.artifacts['weights'])
        
    def predict(self, context, model_input:str, conf:float=0.25, mode='detect'):
        results = self.model.predict(model_input, conf=conf) if mode=="detect" else self.model.track(model_input, conf=conf, persist=True)
        return results
    
    def infer(self, model_input:str, conf:float=0.25):
        return self.model.predict(model_input, conf=conf)
     
    def track(self, model_input:str, conf:float=0.25, persist=True):
        return self.model.track(model_input, conf=conf, persist=persist)
    
    
def push(model_name, model_path):
    # Start an MLflow run
    with mlflow.start_run() as run:
        yolo_model = ModelWrapper(weights=model_path)
        
        mlflow.pyfunc.log_model(
            artifact_path="model",
            python_model=yolo_model,
            conda_env={
                'name': 'mlflow-env',
                'channels': ['defaults', 'conda-forge'],
                'dependencies': [
                    'python=3.8',
                    'pip',
                    {
                        'pip': [
                            'mlflow',
                            'torch',
                            'ultralytics',
                        ],
                    },
                ],
            },
        )
        
        # Optionally, log other parameters and metrics
        mlflow.log_param("model_type", "YOLOv8")
        mlflow.log_metric("mAP50", 0.90)

    result = mlflow.register_model(
        model_uri=f"runs:/{run.info.run_id}/model",
        name=model_name
    )
    
    new_version = result.version
    client.set_registered_model_alias(
        name=model_name,
        alias=ALIAS_PROD,
        version=new_version
    )
        
    model = client.get_registered_model(model_name)
    print_model_info(model)

    model = client.get_model_version_by_alias(model_name, alias=ALIAS_PROD)
    print_model_version_info(model)
    
    artifact_uri = client.get_model_version_download_uri(model_name, model.version)
    print(f"Download URI: {artifact_uri}")
    
def pull(model_name, run_id:str=None):
    if run_id:
        model_uri = f'runs:/{run_id}/model'
    else:
        model_version = client.get_model_version_by_alias(model_name, alias=ALIAS_PROD)
        model_uri = model_uri = f"models:/{model_name}/{model_version.version}"
        
    logging.info(f"MODEL_URI: {model_uri}")
    local_dir = f"/tmp/artifact_downloads/{model_name}"
    if not os.path.exists(local_dir):
        logging.info(f'Model {model_name} does not exist in {local_dir} ! Downloading ...')
        
        os.makedirs(local_dir)
        local_path = mlflow.artifacts.download_artifacts(artifact_uri=model_uri, dst_path=local_dir)
        model = mlflow.pyfunc.load_model(local_path)
        logging.info(f'Model {local_dir} loaded successfully !')
        return model
    
    logging.info(f'Model {model_name} Found ! Checking model Version')
    model = mlflow.pyfunc.load_model(local_dir)
    
    logging.info(f"Current Model Version of {model.metadata.run_id} vs Last Model Version {model_version.run_id}")
    if model.metadata.run_id == model_version.run_id:
        return model
    
    logging.info(f"Currend Model Version {model.metadata.run_id} does not match! Downloading new model version {model_version.run_id}")
    local_path = mlflow.artifacts.download_artifacts(artifact_uri=model_uri, dst_path=local_dir)
    return mlflow.pyfunc.load_model(model_uri)
