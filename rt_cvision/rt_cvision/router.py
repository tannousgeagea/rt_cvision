class ConfigRouter:
    """
    Router to direct certain apps to the 'config' database
    and all other apps to the 'default' database
    """
    config_apps = {'configure'}
    
    # Apps that should only go to default database
    default_only_apps = {
        'admin', 'auth', 'contenttypes', 'sessions', 
        'users', 'tenants', 'metadata', 'data_reader', 'impurity'
    }

    def db_for_read(self, model, **hints):
        if model._meta.app_label in self.config_apps:
            return 'config'
        return 'default'

    def db_for_write(self, model, **hints):
        if model._meta.app_label in self.config_apps:
            return 'config'
        return 'default'

    def allow_relation(self, obj1, obj2, **hints):
        """Allow relations if both models are in the same database"""
        db_set = {'default', 'config'}
        if obj1._state.db in db_set and obj2._state.db in db_set:
            return obj1._state.db == obj2._state.db
        return None

    def allow_migrate(self, db, app_label, model_name=None, **hints):
        """
        Ensure that certain apps' models get created on the right database.
        """
        if app_label in self.config_apps:
            # configure app should only migrate to config database
            return db == 'config'
        elif app_label in self.default_only_apps:
            # All other specified apps should only migrate to default database
            return db == 'default'
        
        # For any other apps not specified, allow default behavior
        return db == 'default'