

'''
Search for available BEARs
'''

from pybear import Manager

from main_controller.config import PORT, BAUDRATE

# Define ID search range
id_range = range(0, 9)

bear = Manager.BEAR(port=PORT, baudrate=BAUDRATE)
bear_list = []
found = False
for i in id_range:
    m_id = i
    print("Pinging BEAR with ID %d" % m_id)
    data = bear.ping(m_id)[0][1]
    if data is not None:
        print("Found BEAR with ID %d." % m_id)
        found = True
        bear_list.append(m_id)
if found:
    print("Search done. Total of %d BEARs found and their IDs are:\n" % len(bear_list))
    print(bear_list)
else:
    print("Search done. No BEAR found.")

