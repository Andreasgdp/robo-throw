# grab all data stored with the following format from a csv file:
# id,success,testType,failedAt,deviation,objPosX,objPosY,goalPosX,goalPosY,j1,j2,j3,j4,j5,j6,x,y,z,q1,q2,q3
# 1,0,"Ping Pong Ball",Throw,3,0.411397,0.554781,0.0957447,0.0840909,1.50866,-1.73169,1.56228,-2.95025,-0.649969,-1.57819,0.152187,-0.381015,0.669118,1.81958,-0.688656,1.8023
# 73,0,"Ping Pong Ball: Reduced throwTime",Throw,12,0.452623,0.552594,0.205167,0.0818182,1.57388,-1.65657,1.44778,-2.91317,-0.565851,-1.57542,0.180794,-0.393143,0.688459,1.91315,-0.
# 74,1,"Consistency test: Ping Pong Ball",,-1,0.452616,0.550293,0.104863,0.138636,2.1324,-2.03595,2.31482,-1.86485,-1.55215,-1.37832,0.244496,-0.180027,0.280357,-2.58145,1.77171,-0.0344134
# 86,1,Golfball,,-1,0.404258,0.449013,0.100304,0.0863636,2.13237,-2.03592,2.31485,-1.86486,-1.55216,-1.37835,0.244489,-0.180037,0.28034,-2.58144,1.77172,-0.0343544
# 102,1,"Golfball: reduced throwTime",,-1,0.502906,0.504435,0.0866261,0.295455,0,0,0,0,0,0,0,0,0,0,0,0
# 117,0,"Consistency test: Golfball",Throw,4,0.454914,0.552601,0.0911854,0.0818182,0,0,0,0,0,0,0,0,0,0,0,0

# import needed libraries
import csv
import matplotlib.pyplot as plt
import numpy as np

# Get data from csv file
def get_data(file_name):
    data = []
    with open(file_name, "r") as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        for row in reader:
            data.append(row)
    return data


# analyze data based on testType
def segmentate_data(data, test_type):
    # initialize variables
    success = 0
    failed = 0
    success_list = []
    failed_list = []
    # iterate through data
    for row in data:
        # check if testType is in row
        if test_type in row:
            # check if success is in row
            if row[1] == "1":
                success += 1
                success_list.append(row)
            # check if failed is in row
            elif row[1] == "0":
                failed += 1
                failed_list.append(row)

    return {
        "success": success,
        "failed": failed,
        "success_list": success_list,
        "failed_list": failed_list,
    }


def generate_deviation_graphs_based_on_test_type(segmented_data):
    figures = []

    # iterate through analyzed data
    for key, value in segmented_data.items():
        # initialize variables
        success_list = value["success_list"]
        failed_list = value["failed_list"]
        # initialize graph and save to figures
        plt.figure()
        plt.title(key)
        plt.xlabel("id")
        plt.ylabel("deviation")
        # iterate through success_list
        for row in success_list:
            # get deviation
            deviation = float(row[4])
            # add deviation to graph
            plt.scatter(row[0], deviation, c="green")
        # iterate through failed_list
        for row in failed_list:
            # get deviation
            deviation = float(row[4])
            # add deviation to graph
            plt.scatter(row[0], deviation, c="red")
        # save graph to figures
        figures.append(plt.gcf())


def generate_deviation_graphs_for_each_object(segmented_data):
    analyzed_data = {
        "Ping Pong Ball": {"deviations": []},
        "Golfball": {"deviations": []},
    }

    # append the deviation for all data from Ping Pong Ball and Golfball
    for row in segmented_data["Ping Pong Ball"]["failed_list"]:
        analyzed_data["Ping Pong Ball"]["deviations"].append(float(row[4]))
    for row in segmented_data["Golfball"]["failed_list"]:
        analyzed_data["Golfball"]["deviations"].append(float(row[4]))

    # plot the deviation for Ping Pong Ball and Golfball
    plt.figure()
    plt.title("Ping Pong Ball and Golfball deviation on failed throws")
    plt.xlabel("id")
    plt.ylabel("deviation")
    plt.scatter(
        [
            "Ping Pong Ball"
            for i in range(len(analyzed_data["Ping Pong Ball"]["deviations"]))
        ],
        analyzed_data["Ping Pong Ball"]["deviations"],
        c="green",
    )
    plt.scatter(
        ["Golfball" for i in range(len(analyzed_data["Golfball"]["deviations"]))],
        analyzed_data["Golfball"]["deviations"],
        c="blue",
    )


def generate_graph_over_deviation_of_target_related_to_distance_of_throw(data):
    # generate plots for the deviation of target in relation to the throw_position
    # TODO: verify that this is the correct pos
    # throw pos (x, y)
    throw_pos_coords = (0.498719, 0.41015)
    data.pop(0)
    plt.figure()
    plt.title("Distance of throw in relation to deviation from target")
    plt.xlabel("distance from throw pos")
    plt.ylabel("deviation")
    deviations = []
    distances = []

    # loop through all data
    for row in data:
        # get deviation
        deviation = float(row[4])
        # get target pos
        target_pos_coords = (float(row[7]), float(row[8]))
        # calculate distance between target and throw pos
        distance = (
            (target_pos_coords[0] - throw_pos_coords[0]) ** 2
            + (target_pos_coords[1] - throw_pos_coords[1]) ** 2
        ) ** 0.5
        if deviation <= 0 or distance <= 0.17:
            continue
        deviations.append(deviation)
        distances.append(distance)

    # add distance and deviation to graph
    plt.scatter(distances, deviations, c="blue")
    # plot the data itself
    plt.plot(distances, deviations, "o")

    # calc the trendline
    z = np.polyfit(distances, deviations, 1)
    p = np.poly1d(z)
    plt.plot(distances, p(distances), "r--")
    # the line equation:
    print("y=%.6fx+(%.6f)" % (z[0], z[1]))


if __name__ == "__main__":
    data = get_data("data.csv")
    segmented_data = {
        "Ping Pong Ball": segmentate_data(data, "Ping Pong Ball"),
        "Ping Pong Ball: Reduced throwTime": segmentate_data(
            data, "Ping Pong Ball: Reduced throwTime"
        ),
        "Consistency test: Ping Pong Ball": segmentate_data(
            data, "Consistency test: Ping Pong Ball"
        ),
        "Golfball": segmentate_data(data, "Golfball"),
        "Golfball: reduced throwTime": segmentate_data(
            data, "Golfball: reduced throwTime"
        ),
        "Consistency test: Golfball": segmentate_data(
            data, "Consistency test: Golfball"
        ),
    }

    generate_deviation_graphs_based_on_test_type(segmented_data)
    generate_deviation_graphs_for_each_object(segmented_data)
    generate_graph_over_deviation_of_target_related_to_distance_of_throw(data)

    plt.show()