import os
import sys

# make the applier package importable when running pytest from anywhere
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
