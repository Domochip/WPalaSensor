from pprint import pprint
import re
from os.path import join, dirname

Import("env")

def extract_macro_value(file_path, macro_name):
    with open(file_path, 'r') as file:
        content = file.read()
        pattern = rf'#define\s+{macro_name}\s+"?([^"\s]+)"?'
        match = re.search(pattern, content)
        if match:
            return match.group(1)
        else:
            raise ValueError(f"{macro_name} not found in {file_path}")

model = extract_macro_value(r'./src/Main.h', 'CUSTOM_APP_MODEL')
version = extract_macro_value(r'./src/Main.h', 'VERSION_NUMBER')

platform = env.GetProjectOption("platform")
esp32suffix = ".esp32" if platform == "espressif32" else ""

env.Replace(PROGNAME=f"{model}{esp32suffix}.{version}")