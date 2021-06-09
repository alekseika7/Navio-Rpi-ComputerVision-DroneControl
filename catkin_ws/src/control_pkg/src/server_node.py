#!python3

from collections import Counter
from socketserver import BaseRequestHandler, TCPServer
import rospy
from std_msgs.msg import String


HOST = "localhost"
PORT = 8080


def accumulate_commands(commands):
    counts = Counter(commands)
    real_label = counts.most_common(1)[0][0]
    return real_label


def parse_label(req):
    start_index = req.rfind("=")
    pred_label = ""
    for i in range(start_index + 1, len(req)):
        next_char = req[i]
        if next_char != " ":
            pred_label += next_char
        else:
            break
    return pred_label


class RequestHandler(BaseRequestHandler):

    def handle(self):
        data = self.request.recv(1024).decode().strip()
        prediction = parse_label(data)

        response = "HTTP/1.1 200 OK\r\n\r\n"
        self.request.sendall(response.encode())

        global label_container

        if len(label_container) < 1:
            label_container.append(prediction)
        else:
            label = accumulate_commands(label_container)
            pub.publish(label)
            rospy.loginfo(label)
            label_container = []


if __name__ == '__main__':
    try:
        rospy.init_node('server_node')
        pub = rospy.Publisher('/command_from_bs', String, queue_size=1)
        rospy.loginfo('Ready to recieve commands!')
        rospy.loginfo("Server started http://%s:%s" % (HOST, PORT))
        server = TCPServer((HOST, PORT), RequestHandler)
        label_container = []
        while True:
            server.serve_forever()
        server.close()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

