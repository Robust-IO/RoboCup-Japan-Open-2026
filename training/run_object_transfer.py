"""Common entry for the verified apple route and the new continuous can trial."""
import argparse
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__,add_help=False)
    p.add_argument('--object',required=True,choices=('apple','canned_juice','ground_pepper','toy_penguin'))
    args,remaining=p.parse_known_args()
    if args.object=='canned_juice':
        from can_transfer_automation import main as can_main
        can_main(remaining)
    elif args.object=='toy_penguin':
        from penguin_transfer_automation import main as penguin_main
        penguin_main(remaining)
    elif args.object=='ground_pepper':
        from pepper_transfer_automation import main as pepper_main
        pepper_main(remaining)
    else:
        from apple_transfer_automation import main as apple_main
        sys.argv=[sys.argv[0],*remaining];apple_main()


if __name__=='__main__':main()
