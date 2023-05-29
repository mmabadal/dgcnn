import os
import re
import argparse
import sys


def get_header(file):
    header = []
    while True:
        ln = file.readline().strip()
        header.append(ln)
        if ln.startswith(b'end'):
            break
    return header


def get_data(file):
    data = []
    while True:
        ln = file.readline()
        if ln == b'':
            break
        data.append(ln)
    return data


def main():

    # python ply_to_txt.py --path_in ../data_test2/ply/ --path_out ../data_test2/txt #

    parser = argparse.ArgumentParser()
    parser.add_argument('--path_in', help='path to the ply dataset folder.')
    parser.add_argument('--path_out', help='path to the out dataset folder.')
    parsed_args = parser.parse_args(sys.argv[1:])

    path_in = parsed_args.path_in
    path_out = parsed_args.path_out

    if not os.path.exists(path_out):
        os.makedirs(path_out)

        if not os.path.exists(os.path.join(path_out)):
            os.makedirs(os.path.join(path_out))

    for file in sorted(os.listdir(path_in)):

        if re.search("\.(ply)$", file):  # if the file is a ply

            file_in = os.path.join(path_in, file)
            name = os.path.splitext(file)[0]
            file_out = os.path.join(path_out, name + '.txt')

            with open(file_in, 'rb') as f:
                header = get_header(f)
                data = get_data(f)

            f = open(file_out, 'w')

            for row in data:
                row = row.decode("utf-8")
                f.write(row)



if __name__ == "__main__":
    main()