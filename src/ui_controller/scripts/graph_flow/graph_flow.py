#! /usr/bin/python3
# -*- coding: utf-8 -*-

# import sys, os
# # 指定要添加的文件夹，使用os.path.join()函数以正确的方式构造文件名
# from os import path
# def add_path_to_sys_path(folder_path):
#     # 将文件夹添加到sys.path中
#     if folder_path not in sys.path:
#         sys.path.append(folder_path)

# # 添加文件夹作为模块搜索路径
# prj_dir = os.path.dirname(os.path.abspath(__file__)) # 获取当前文件所在文件夹
# folders = [os.path.join(prj_dir, 'src'), 
#            os.path.join(prj_dir, 'res/renamedata'),
#            os.path.join(prj_dir, 'ui'),
#            os.path.join(prj_dir, 'utils'),
#            os.path.join(prj_dir, 'logs'),
#            os.path.join(prj_dir, 'settings'),]

# for folder in folders:
#     for root, dirs, files in os.walk(folder):
#         add_path_to_sys_path(root)
#         for dir in dirs:
#             add_path_to_sys_path(dir)

from pathlib import Path

from Qt import QtCore

from NodeGraphQt import (
    NodeGraph,
    PropertiesBinWidget,
    NodesTreeWidget,
    NodesPaletteWidget
)

# import example nodes from the "nodes" sub-package
from scripts.graph_flow.nodes import basic_nodes, custom_ports_node, group_node, widget_nodes

BASE_PATH = Path(__file__).parent.resolve()

class GraphFlow:
    def __init__(self):
        pass

    def init_graph(self, verticalLayout_flow):
        """"""
        # create graph controller.
        graph = NodeGraph()
        graph_widget = graph.widget
        # graph_widget.resize(1100, 800)
        # graph_widget.show()
        verticalLayout_flow.addWidget(graph_widget)

        # set up context menu for the node graph.
        hotkey_path = Path(BASE_PATH, 'hotkeys', 'hotkeys.json')
        graph.set_context_menu_from_file(hotkey_path, 'graph')

        # registered example nodes.
        graph.register_nodes([
            basic_nodes.BasicNodeA,
            basic_nodes.BasicNodeB,
            basic_nodes.CircleNode,
            custom_ports_node.CustomPortsNode,
            group_node.MyGroupNode,
            widget_nodes.DropdownMenuNode,
            widget_nodes.TextInputNode,
            widget_nodes.CheckboxNode
        ])

        # 创建节点
        # create node with custom text color and disable it.
        self.n_basic_a = graph.create_node('nodes.basic.BasicNodeB', name='Object detection')
        self.n_basic_a.set_icon(Path(BASE_PATH, 'star.png'))
        # n_basic_a.set_disabled(True)
        
        # create node and set a custom icon.
        self.n_basic_b = graph.create_node('nodes.basic.BasicNodeB', name='Bolt position calculation')
        self.n_basic_b.set_icon(Path(BASE_PATH, 'star.png'))

        self.n_basic_c = graph.create_node('nodes.basic.BasicNodeB', name='Arm operation')
        self.n_basic_c.set_icon(Path(BASE_PATH, 'star.png'))

        # 连接节点
        self.n_basic_a.set_output(0, self.n_basic_b.input(0))
        self.n_basic_b.set_output(0, self.n_basic_c.input(0))

        # # auto layout nodes.
        graph.auto_layout_nodes()

        # fit nodes to the viewer.
        graph.clear_selection()
        graph.fit_to_selection()

        # create a node properties bin widget.
        properties_bin = PropertiesBinWidget(node_graph=graph)
        properties_bin.setWindowFlags(QtCore.Qt.Tool)

        # example show the node properties bin widget when a node is double-clicked.
        def display_properties_bin(node):
            if not properties_bin.isVisible():
                properties_bin.show()

        # wire function to "node_double_clicked" signal.
        graph.node_double_clicked.connect(display_properties_bin)

    def execute_nodes(self,):
        """
        递归执行节点的处理函数。
        """
        self.n_basic_a.process()
        self.n_basic_b.process()
        self.n_basic_c.process()