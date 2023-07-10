"""
MIT LICENSE

Copyright (C) 2023 Murilo Marques Marinho (www.murilomarinho.info)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import os
import json
from pathlib import Path


def make_or_get_default_user_configuration() -> dict:
    """
    Gets the contents of the configuration file whose path is defined by
    get_configuration_file_path()
    or creates a new default user configuration at that same location.
    :return: a dict equivalent to the contents of the configuration file.
    """
    if not os.path.isfile(get_configuration_file_path()):
        default_user_configuration = {
            "mantainer_name": 'TODO',
            "mantainer_email": 'TODO@TO.DO',
            "license": 'TODO'
        }

        save_user_configuration(default_user_configuration)

    return get_user_configuration()


def get_configuration_file_path() -> Path:
    """
    Obtains the configuration file path, defined by
    ``~/.notros2/config.json``.
    The directory is defined in
    make_or_get_user_configuration_directory()
    while the filename is defined herein.
    :return: a pathlib.Path object.
    """
    return make_or_get_user_configuration_directory() / Path("config.json")


def get_user_configuration() -> dict:
    """
    Gets the contents of the configuration file whose path is defined by
    get_configuration_file_path()
    :return: a dict with the contents of the file.
    """
    with open(get_configuration_file_path(), 'r') as configuration_file:
        config = json.load(configuration_file)
    return config


def save_user_configuration(config: dict) -> None:
    """
    Saves a dict into the configuration file defined by
    get_configuration_file_path()
    :param config: The dict to be saved.
    :return: None.
    """
    with open(get_configuration_file_path(), 'w') as configuration_file:
        configuration_file.write(json.dumps(config))


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
