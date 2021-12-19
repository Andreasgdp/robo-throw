# grab all data stored with the following format from a csv file:
# id,success,testType,failedAt,deviation,objPosX,objPosY,goalPosX,goalPosY,j1,j2,j3,j4,j5,j6,x,y,z,q1,q2,q3,totalThrowTime,calibImgTime,findObjTime,pathCalcTime,grabTime,throwTime,apiLogTime
# 50,0,"Ping Pong Ball: Reduced velocity",Throw,15,0.450312,0.545686,0.0911854,0.613636,1.44522,-2.02507,1.95945,-3.05429,-1.49734,-1.57937,0.0784704,-0.301534,0.590415,1.18768,-1.24457,1.15357,21547.3,-1,9195.39,0.429202,5638.67,13546.2,12.1396
# 67,1,"Ping Pong Ball: Reduced throwTime",,-1,0.45261,0.547993,0.191489,0.720455,0,0,0,0,0,0,0,0,0,0,0,0,15803.4,-1,201617,0.338933,7293.74,203523,32.807
# 74,1,"Consistency test: Ping Pong Ball",,-1,0.452616,0.550293,0.104863,0.138636,2.1324,-2.03595,2.31482,-1.86485,-1.55215,-1.37832,0.244496,-0.180027,0.280357,-2.58145,1.77171,-0.0344134,17200.4,-1,6882.11,0.261349,7289.45,8755.21,32.7918
# 86,1,Golfball,,-1,0.404258,0.449013,0.100304,0.0863636,2.13237,-2.03592,2.31485,-1.86486,-1.55216,-1.37835,0.244489,-0.180037,0.28034,-2.58144,1.77172,-0.0343544,21839.3,-1,54222.1,0.501057,7054.57,56122.9,10.8028
# 133,1,"Consistency test: Golfball",,-1,0.55824,0.596654,0.132219,0.181818,0,0,0,0,0,0,0,0,0,0,0,0,28379.2,-1,4716.86,0.434768,7847.02,6580.51,10.1525
# 147,1,"Consistency test2: Golfball",,-1,0.553531,0.559802,0.145897,0.186364,2.13238,-2.03593,2.31486,-1.86491,-1.55214,-1.37832,0.244493,-0.180032,0.280342,-2.58142,1.77173,-0.0344387,18963.9,-1,14054.2,0.250993,7735.4,15929.2,31.9517
# 165,1,"Consistency test3: Golfball",,-1,0.418139,0.506501,0.12766,0.620455,0,0,0,0,0,0,0,0,0,0,0,0,27415.5,-1,5691.36,0.236935,7119.93,7588.47,32.8105
# 175,1,"Consistency test4: Golfball",,-1,0.26021,0.460186,0.0296353,0.402273,0,0,0,0,0,0,0,0,0,0,0,0,17165.5,-1,13663.4,0.473808,7026.71,15584.9,35.9159

# import needed libraries
import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


def deviation_corrector(deviation: float):
    if deviation <= 0:
        return deviation
    return deviation + 4.5


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


def generate_deviation_graphs_for_each_object(segmented_data):
    analyzed_data = {
        "Ping Pong Ball": {"deviations": []},
        "Golfball": {"deviations": []},
    }

    # iterate through analyzed data
    for key, value in segmented_data.items():
        # append the deviation for all data from Ping Pong Ball and Golfball
        if "Ping Pong Ball" in key:
            for row in value["failed_list"]:
                deviation = deviation_corrector(float(row[4]))
                if deviation <= 0:
                    continue
                analyzed_data["Ping Pong Ball"]["deviations"].append(deviation)

        if "Golfball" in key:
            for row in value["failed_list"]:
                deviation = deviation_corrector(float(row[4]))
                if deviation <= 0:
                    continue
                analyzed_data["Golfball"]["deviations"].append(deviation)

    # plot the deviation for Ping Pong Ball and Golfball
    plt.figure()
    plt.title("Deviation on failed throws")
    plt.xlabel("id")
    plt.ylabel("deviation [cm]")
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


def generate_graphs_for_success_rate_for_each_object(segmented_data):
    analyzed_data = {
        "Ping Pong Ball": {"success": 0, "failed": 0},
        "Golfball": {"success": 0, "failed": 0},
    }

    # iterate through analyzed data
    for key, value in segmented_data.items():
        # append the success and failed for all data from Ping Pong Ball and Golfball
        if "Ping Pong Ball" in key:
            analyzed_data["Ping Pong Ball"]["success"] += value["success"]
            analyzed_data["Ping Pong Ball"]["failed"] += value["failed"]

        if "Golfball" in key:
            analyzed_data["Golfball"]["success"] += value["success"]
            analyzed_data["Golfball"]["failed"] += value["failed"]

    print(
        "ping pong ball success rate: "
        + str(
            analyzed_data["Ping Pong Ball"]["success"]
            / (
                analyzed_data["Ping Pong Ball"]["success"]
                + analyzed_data["Ping Pong Ball"]["failed"]
            )
        )
    )
    print(
        "golfball success rate: "
        + str(
            analyzed_data["Golfball"]["success"]
            / (
                analyzed_data["Golfball"]["success"]
                + analyzed_data["Golfball"]["failed"]
            )
        )
    )


def generate_graph_over_deviation_of_target_related_to_distance_of_throw(data):
    # generate plots for the deviation of target in relation to the throw_position
    # throw pos (x, y)
    throw_pos_coords = (0.498719, 0.41015)
    data.pop(0)
    plt.figure()
    plt.title("Deviation from target in relation to distance of throw")
    plt.xlabel("distance from throw pos [cm]")
    plt.ylabel("deviation [cm]")
    deviations = []
    distances = []

    # loop through all data
    for row in data:
        # get deviation
        deviation = deviation_corrector(float(row[4]))
        # get target pos
        target_pos_coords = (float(row[7]), float(row[8]))
        # calculate distance between target and throw pos
        distance = (
            (target_pos_coords[0] - throw_pos_coords[0]) ** 2
            + (target_pos_coords[1] - throw_pos_coords[1]) ** 2
        ) ** 0.5
        if distance <= 0.17:
            continue
        deviation = max(deviation, 0)
        deviations.append(deviation)
        distances.append(distance * 100)

    # add distance and deviation to graph
    plt.scatter(distances, deviations, c="blue")
    # plot the data itself
    plt.plot(distances, deviations, "o")

    # calc the trendline
    # z = np.polyfit(distances, deviations, 1)
    # p = np.poly1d(z)
    # plt.plot(np.linspace(distances[0], distances[-1], 159), p(distances), "r--")

    # define the true objective function
    def objective(x, a, b, c):
        return a * x + b * x ** 2 + c

    # curve fit
    popt, _ = curve_fit(objective, distances, deviations)
    # summarize the parameter values
    a, b, c = popt
    print("y = %.5f * x + %.5f * x^2 + %.5f" % (a, b, c))
    # define a sequence of inputs between the smallest and largest known inputs
    x_line = np.arange(min(distances), max(distances), 1)
    # calculate the output for the range
    y_line = objective(x_line, a, b, c)
    # create a line plot for the mapping function
    plt.plot(x_line, y_line, "--", color="red")


def calculate_grab_object_success_rate(data):
    # calculate the success rate of grabbing an object
    grab_success_rate = 0
    grab_failed_rate = 0
    success_rate = 0
    failed_rate = 0
    for row in data:
        if row[3] in ["Moving above object", "Grabbing object"]:
            if row[3] == "Grabbing object":
                grab_failed_rate += 1
            failed_rate += 1
        else:
            success_rate += 1
            grab_success_rate += 1
    print(
        "grabbing object success rate: "
        + str(grab_success_rate / (grab_success_rate + grab_failed_rate))
    )
    print(
        "Overall success rate of moving to object and grabbing: "
        + str(success_rate / (success_rate + failed_rate))
    )


def calculate_avg_time_for_each_event(data):
    cols = [
        "id",
        "success",
        "testType",
        "failedAt",
        "deviation",
        "objPosX",
        "objPosY",
        "goalPosX",
        "goalPosY",
        "j1",
        "j2",
        "j3",
        "j4",
        "j5",
        "j6",
        "x",
        "y",
        "z",
        "q1",
        "q2",
        "q3",
        "totalThrowTime",
        "calibImgTime",
        "findObjTime",
        "pathCalcTime",
        "grabTime",
        "throwTime",
        "apiLogTime",
    ]
    # calculate the average time for each event
    avg_time_for_each_event = {
        "findObjTime": [],
        "pathCalcTime": [],
        "grabTime": [],
        "throwTime": [],
        "apiLogTime": [],
    }
    for row in data:
        for index, elm in enumerate(cols):
            if elm in avg_time_for_each_event and float(row[index]) >= 0:
                avg_time_for_each_event[elm].append(float(row[index]))

    for key, value in avg_time_for_each_event.items():
        if len(value) <= 0:
            avg_time_for_each_event[key] = -1
        else:
            avg_time_for_each_event[key] = sum(value) / len(value) / 1000

    avg_time_for_each_event["totalThrowTime"] = (
        avg_time_for_each_event["grabTime"]
        + avg_time_for_each_event["pathCalcTime"]
        + avg_time_for_each_event["findObjTime"]
        + avg_time_for_each_event["throwTime"]
    )

    avg_time_for_each_event["calibImgTime"] = 10

    # sort the avg_time_for_each_event
    avg_time_for_each_event = {
        k: v
        for k, v in sorted(avg_time_for_each_event.items(), key=lambda item: item[1])
    }

    # show graph of avg time for each event
    plt.figure()
    plt.title("Average time for each event")
    plt.xlabel("event")
    plt.ylabel("time [s]")
    plt.bar(
        avg_time_for_each_event.keys(),
        avg_time_for_each_event.values(),
    )

    print(avg_time_for_each_event)


def remove_data_containing_object(data, object: str):
    data = [row for row in data if object not in row[2]]
    return data


if __name__ == "__main__":
    data = get_data("data2.csv")
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
        "Consistency test2: Golfball": segmentate_data(
            data, "Consistency test2: Golfball"
        ),
        "Consistency test3: Golfball": segmentate_data(
            data, "Consistency test3: Golfball"
        ),
        "Consistency test4: Golfball": segmentate_data(
            data, "Consistency test4: Golfball"
        ),
    }

    generate_deviation_graphs_for_each_object(segmented_data)
    generate_graphs_for_success_rate_for_each_object(segmented_data)

    generate_graph_over_deviation_of_target_related_to_distance_of_throw(data)
    generate_graph_over_deviation_of_target_related_to_distance_of_throw(
        remove_data_containing_object(data, "Golfball")
    )
    generate_graph_over_deviation_of_target_related_to_distance_of_throw(
        remove_data_containing_object(data, "Ping Pong Ball")
    )
    calculate_grab_object_success_rate(data)
    calculate_avg_time_for_each_event(data)

    plt.show()