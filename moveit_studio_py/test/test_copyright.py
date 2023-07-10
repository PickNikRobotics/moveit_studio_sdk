# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from ament_copyright.main import main
import pytest
import unittest


@pytest.mark.copyright
@pytest.mark.linter
class TestCopyright(unittest.TestCase):
    def test_copyright(self):
        rc = main(argv=[".", "test"])
        self.assertTrue(rc == 0, msg="Found errors")
