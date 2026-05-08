import sys

from milton_final_project.navigate_to_label import main as navigate_to_label_main


def main(args=None):
    forwarded_args = ['--start']
    if args is None:
        forwarded_args.extend(sys.argv[1:])
    else:
        forwarded_args.extend(args)
    navigate_to_label_main(forwarded_args)


if __name__ == '__main__':
    main()
