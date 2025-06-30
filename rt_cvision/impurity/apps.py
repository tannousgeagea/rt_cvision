from django.apps import AppConfig


class ImpurityAppConfig(AppConfig):
    default_auto_field = 'django.db.models.BigAutoField'
    name = 'impurity'

    # def ready(self):
    #     # Import the signal handlers
    #     import impurity.signals