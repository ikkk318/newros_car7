#!/usr/bin/env python3
"""
agv_lift —— 搬货服务（顶升 / 下降二合一）。

本包只有一个节点 cargo_node，提供 /agv/cargo 服务并维护 /agv/cargo/busy 标志。
它是全项目唯一与"搬货机器"打交道的地方：接真实硬件时只改这一个文件的服务实现，
外部接口（服务名、字段、话题）保持不变。
"""
