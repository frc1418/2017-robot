import pytest
import os
cwd = os.path.dirname(os.path.realpath(__file__))
for root, subFolders, files in os.walk(cwd + '/../robot'):
	for file in files:
		if file.endswith('.py'):
			with open(os.path.join(root, file), 'r') as f:
				for line in f:
					assert not line.lstrip().startswith('print(')
