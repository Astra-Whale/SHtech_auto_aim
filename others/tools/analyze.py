import re
import matplotlib.pyplot as plt
import csv


def main():
    axis_Y = []
    axis_X = []
    noise_Y = []
    noise_X = []
    noise_frame = []
    frame = []
    num = 0
    noise_num = 0
    log = input("Please type in log name.\n")
    path = "../../log/" + log
    while True:
        name = input(
            "Please type in the name you want to analyse or just exit with 'q'.\n")
        if(name == 'q'):
            break
        else:
            pattern = name + r":\((.*),(.*)\)"
            recongnize = re.compile(pattern)
            f = open(path, 'r')
            while True:
                line = f.readline()
                if line:
                    search = recongnize.search(line)
                    if search != None:
                        axis_Y.append(search.group(1))
                        frame.append(search.group(2))
                        axis_X.append(num)
                        num += 1
                else:
                    if num == 0:
                        print("Wrong name type! Please try again!")
                        break
                    path_name = "../tools/" + name + '.csv'
                    with open(path_name, 'w') as fi:
                        csv_write = csv.writer(fi)
                        csv_head = [name, 'frame']
                        csv_write.writerow(csv_head)
                        for i in range(num):
                            csv_write.writerow([axis_Y[i], frame[i]])
                    fi.close()
                    flag = ' '
                    flag = input('Do you want to find the noise(y/n)\n')
                    if flag == 'n':
                        axis_Y = [float(i) for i in axis_Y]
                        plt.scatter(axis_X, axis_Y, s=3)
                        plt.show()
                    elif flag == 'y':
                        A = float(
                            input('Please type in the max amplitude you can tolerate\n'))
                        temp = axis_Y[0]
                        for i in range(num - 1):
                            if ((float(axis_Y[i + 1]) - float(temp) > A) or (float(temp) - float(axis_Y[i + 1]) > A)):
                                noise_Y.append(axis_Y[i + 1])
                                noise_frame.append(frame[i + 1])
                                noise_X.append(axis_X[i + 1])
                                noise_num += 1
                            else:
                                temp = axis_Y[i + 1]
                        path_name = "../tools/" + name + '_noise.csv'
                        with open(path_name, 'w') as fn:
                            csv_write = csv.writer(fn)
                            csv_head = [name, 'frame']
                            csv_write.writerow(csv_head)
                            for i in range(noise_num):
                                csv_write.writerow(
                                    [noise_Y[i], noise_frame[i]])
                        fn.close()
                        axis_Y = [float(i) for i in axis_Y]
                        noise_Y = [float(i) for i in noise_Y]
                        plt.scatter(axis_X, axis_Y, s=3)
                        plt.scatter(noise_X, noise_Y, s=3)
                        plt.show()

                    axis_Y = []
                    axis_X = []
                    frame = []
                    noise_Y = []
                    noise_X = []
                    noise_frame = []
                    num = 0
                    noise_num = 0
                    break


if __name__ == "__main__":
    main()
