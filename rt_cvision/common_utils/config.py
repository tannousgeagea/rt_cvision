import os
import yaml
import json
import logging
logging.getLogger().setLevel(logging.INFO)

class CFG:
    def __init__(self, config_file, show=False, skip_dict=False):
        self.CFG = {}
        self.config_file = config_file
        self.skip_dict = skip_dict
        self.read_cfg()
        self.read_env_variables()

        if show:
            logging.info('PARAMETERS:')
            for key, value in self.CFG.items():
               logging.info(f"{key:25} \t {value}")

    def read_cfg(self):
        self.CFG = {}
        try:
            cfg = yaml.safe_load(
                open(self.config_file, 'r')
            )

            for pkey, pvalues in cfg.items():
                if type(pvalues) == dict and not self.skip_dict:
                    for skey, svalues in pvalues.items():
                        self.CFG[skey] = cfg[pkey][skey] 
                else:
                    self.CFG[pkey] = cfg[pkey]
        
        except Exception as err:
            logging.error('Unexpected Error while reading config file: %s' %err)

    def read_env_variables(self):
        try:
            cfg = yaml.safe_load(
                open(self.config_file, 'r')
            )

            if 'env_variables' in cfg.keys():
                self.CFG['env_variables'] = {}
                for key, value in cfg['env_variables'].items():
                    os.environ[key] = value
                    self.CFG['env_variables'][key] = value

        except Exception as err:
            logging.error('Unexpected Error while reading environment variable: %s' %err)