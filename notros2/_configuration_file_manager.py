import os
import json
from pathlib import Path


def make_or_get_default_user_configuration() -> dict:
    """
    Gets the contents of the configuration file
    ``~/.notros2/config.json``.
    :return: a dict equivalent to the contents of the configuration file.
    """
    configuration_file_path = make_or_get_user_configuration_directory() / Path("config.yaml")
    if not os.path.isfile(configuration_file_path):

        default_user_configuration = {
            "mantainer_name": 'TODO',
            "mantainer_email": 'TODO@TO.DO',
            "license": 'TODO'
        }

        with open(configuration_file_path, 'w+') as configuration_file:
            configuration_file.write(json.dumps(default_user_configuration))

    # config = {}
    with open(configuration_file_path) as configuration_file:
        config = json.load(configuration_file)

    return config


def make_or_get_user_configuration_directory() -> Path:
    """
    Obtains or creates the user configuration directory, namely
    ``~/.notros2``.
    :return: a pathlib.Path to the folder.
    """
    user_home = os.path.expanduser("~")
    user_config_directory = user_home / Path('.notros2')

    if not os.path.isdir(user_config_directory):
        os.mkdir(user_config_directory)

    return user_config_directory
