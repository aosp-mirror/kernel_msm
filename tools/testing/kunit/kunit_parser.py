import re

kunit_start_re = re.compile('console .* enabled')
kunit_end_re = re.compile('List of all partitions:')

def isolate_kunit_output(kernel_output):
	started = False
	for line in kernel_output:
		if kunit_start_re.match(line):
			started = True
		elif kunit_end_re.match(line):
			break
		elif started:
			yield line

def raw_output(kernel_output):
	for line in kernel_output:
		print(line)

def parse_run_tests(kernel_output):
	for output in isolate_kunit_output(kernel_output):
		print(output)
