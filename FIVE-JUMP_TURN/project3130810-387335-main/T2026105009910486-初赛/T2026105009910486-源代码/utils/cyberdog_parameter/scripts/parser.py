import toml
import os
import sys
import argparse

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

class FileOperation:
    def __init__(self):
        self.dic = dict()

    def __add__(self, other):
        self.dic.update(self.other)
        return self.dic

    def write(self):
        mysql_dic = {"user": "root", "password": "Aa1234"}
        mysql2_dic = {"user1": "root", "password2": "Aa1234"}
        mysql_dic.update(mysql2_dic)
        with open(self.toml_file_path, "w", encoding="utf-8") as fs:
            toml.dump(mysql_dic, fs)

    def read(self, filename):
        self.toml_filename = filename
        with open(filename, "r", encoding="utf-8") as fs:
            t_data = toml.load(fs)
        return t_data

    def generate_hpp(self, data):
        hpp_file  = os.path.split(self.toml_filename)
        module_name = str(hpp_file[-1]).split(".", 1)[0]
        hpp_file = module_name + '.pyx'
        method = "def get_" + str(module_name) + "_data():\n"

        with open(hpp_file, 'w') as f:
            f.write("# cython: language_level=3 \n")
            f.write(method)
            # f.write("\treturn " + "\"" + str(data) + "\"")
            # f.write("\treturn " +  str(toml.dumps(data)))
            f.write("\treturn " + "\'''\n" + str(toml.dumps(data)) + "\t\'''")





