import os

def print_model_info(rm):
    print("--Model--")
    print("name: {}".format(rm.name))
    print("aliases: {}".format(rm.aliases))

def print_models_info(mv):
    for m in mv:
        print(f"name: {m.name}")
        print(f"latest version: {m.version}")
        print(f"run_id: {m.run_id}")
        print(f"current_stage: {m.current_stage}")

def print_model_version_info(mv):
    print("--Model Version--")
    print("Name: {}".format(mv.name))
    print("Version: {}".format(mv.version))
    print("Aliases: {}".format(mv.aliases))