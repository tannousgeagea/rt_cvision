
class ConfigRouter:
    config_apps = {'configure', 'tenants'}

    def db_for_read(self, model, **hints):
        if model._meta.app_label in self.config_apps:
            return 'config'
        return 'default'

    def db_for_write(self, model, **hints):
        if model._meta.app_label in self.config_apps:
            return 'config'
        return 'default'

    def allow_relation(self, obj1, obj2, **hints):
        # Allow if both models are in same DB
        db_obj1 = self.db_for_read(obj1)
        db_obj2 = self.db_for_read(obj2)
        return db_obj1 == db_obj2

    def allow_migrate(self, db, app_label, model_name=None, **hints):
        if app_label in self.config_apps:
            return db == 'config'
        return db == 'default'