import unittest
from composer.workflow.safe_evaluator import SafeEvaluator


class TestSafeEvaluator(unittest.TestCase):
    def test_simple_condition(self):
        context = {'step1': True}
        evaluator = SafeEvaluator(context)
        self.assertTrue(evaluator.eval_expr("step1 == True"))
        self.assertFalse(evaluator.eval_expr("step1 == False"))

    def test_complex_condition(self):
        context = {
            'step1': type('Response', (object,), {'success': True}),
            'step2': type('Response', (object,), {'output': 'desired_state'})
        }
        evaluator = SafeEvaluator(context)
        condition = "step1.success == True and step2.output == 'desired_state'"
        self.assertTrue(evaluator.eval_expr(condition))

    def test_invalid_condition(self):
        context = {}
        evaluator = SafeEvaluator(context)
        with self.assertRaises(ValueError):
            evaluator.eval_expr("import sys")

if __name__ == '__main__':
    unittest.main()
