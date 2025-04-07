import ast

def list_classes_in_file(filepath):
    with open(filepath, "r") as file:
        tree = ast.parse(file.read(), filename=filepath)
    
    classes = [node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)]
    return classes

# Usage
python_file_path = "srunner/scenariomanager/scenarioatomics/atomic_behaviors.py"
print(list_classes_in_file(python_file_path))
