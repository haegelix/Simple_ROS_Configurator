import unittest

import test_paths
import test_runconfigs

loader = unittest.TestLoader()
suite = unittest.TestSuite()

suite.addTests(loader.loadTestsFromModule(test_paths))
suite.addTests(loader.loadTestsFromModule(test_runconfigs))

runner = unittest.TextTestRunner(verbosity=3)
result = runner.run(suite)