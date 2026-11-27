"""Toy-penguin head grasp over the shared furniture transfer engine."""
from pepper_transfer_automation import main as shared_main

def main(argv=None):
    return shared_main(argv,object_name='toy_penguin')

if __name__=='__main__':main()
