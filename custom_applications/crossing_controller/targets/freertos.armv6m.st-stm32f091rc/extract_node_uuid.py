import re
import sys

def ParseModelAndRev(text):
	m = re.match(r"^0x[a-z0-9]+: ([0-9a-f]+) ([0-9a-f]+)$", text)
	if not m:
		raise AttributeError("Model/revision unparseable: got %s" % text)

	model_id = m.group(1)
	assert len(model_id) == 4
	model_id = model_id[1:]  # first char "reserved"
	revision_id = m.group(2)
	model_name = {
		"422": "STM32F303xB/C or STM32F358",
		"438": "STM32F303x6/8 or STM32F328",
		"440": "STM32F05x",
		"442": "STM32F09x",
		"444": "STM32F03x",
		"445": "STM32F04x",
		"446": "STM32F303xD/E or STM32F398xE",
		"448": "STM32F07x",
	}.get(model_id)
	if not model_name:
		raise AttributeError("Unknown model id: %s" % model_id)

	revision_name = {
		"1001": "Z",
		"1003": "Y",
	}.get(revision_id, "REV_UNUSED")
	if not revision_name:
		raise AttributeError("Unknown revision id: %s" % revision_id)

	return model_name, revision_name


def ParseUUID(text):
	m = re.match(r"^0x[a-z0-9]+: ([0-9a-f]+) ([0-9a-f]+)$", text)
	if not m:
		raise AttributeError("Model/revision unparseable: got %s" % text)
	uid = m.group(1) + m.group(2)
	print('Parsed UUID "%s"' % uid, file=sys.stderr)
	return uid

def PickNodeNumber(existing, uid):
	if uid in existing:
		num = existing[uid]
		print('Device already known as node #%s' % num, file=sys.stderr)
		return num, False
	else:
		taken_numbers = set(existing.values())
		for i in range(250):
			if i not in taken_numbers:
				print('Assigning new number %s' % i, file=sys.stderr)
				return i, True
		raise ValueError('Failed to assign new node number')


with open('device_identifiers.output', 'r') as f:
	lines = f.readlines()
	if len(lines) < 2: raise AttributeError("File too short")

	model_name, revision_name = ParseModelAndRev(lines[0].strip())
	uid = ParseUUID(lines[1].strip())

print("MCU: %s, rev %s" % (model_name, revision_name), file=sys.stderr)
print("UID:", uid, file=sys.stderr)

DB_FILE = 'device_database.txt'
		
with open(DB_FILE, 'r') as f:
	existing = {}
	for l in f.readlines():
		existing_uid, node_number = l.strip().split(' ')
		existing[existing_uid] = int(node_number)

print("Existing: %s" % str(existing), file=sys.stderr)

picked_number, is_new = PickNodeNumber(existing, uid)
if is_new:
	with open(DB_FILE, 'a') as f:
		f.write('%s %s\n' % (uid, picked_number))

print(picked_number)