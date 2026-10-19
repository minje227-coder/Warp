"""Run the production Java archive writer without an Android emulator."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

class UploadArchiveTests(unittest.TestCase):
    def test_lossless_stream_and_interruption(self):
        javac=shutil.which('javac');java=shutil.which('java')
        if not javac or not java:self.skipTest('Java compiler/runtime unavailable')
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as d:
            subprocess.run([javac,'-d',d,str(root/'unity/WarpUploadArchive.java'),str(root/'tests/WarpUploadArchiveTest.java')],check=True,capture_output=True,text=True)
            result=subprocess.run([java,'-cp',d,'WarpUploadArchiveTest'],check=True,capture_output=True,text=True)
            self.assertIn('cases passed',result.stdout)
