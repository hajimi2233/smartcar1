"""Only an explicitly labelled final mission end drops its heading constraint."""


def is_end_goal(labels,index):
    return bool(labels and 0<=index<len(labels) and index==len(labels)-1 and labels[index]=='end')
