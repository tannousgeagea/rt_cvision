import mlflow
from core import pull


model = pull(model_name='iserlohn.amk.want:waste.segments')

input_image = 'image.jpg'
results = model.predict(input_image)
unwrapped_model = model.unwrap_python_model()
results = unwrapped_model.track(input_image)