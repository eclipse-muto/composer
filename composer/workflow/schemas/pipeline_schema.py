#
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#

PIPELINE_SCHEMA = {
    "type": "object",
    "properties": {
        "pipelines": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "name": {"type": "string"},
                    "pipeline": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "sequence": {
                                    "type": "array",
                                    "items": {
                                        "type": "object",
                                        "properties": {
                                            "name": {"type": "string"},
                                            "service": {"type": "string"},
                                            "plugin": {"type": "string"},
                                            "condition": {"type": "string"},
                                        },
                                        "required": ["name", "service", "plugin"],
                                    },
                                }
                            },
                            "required": ["sequence"],
                        },
                    },
                    "compensation": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "service": {"type": "string"},
                                "plugin": {"type": "string"},
                            },
                            "required": ["service", "plugin"],
                        },
                    },
                },
                "required": ["name", "pipeline", "compensation"],
            },
        }
    },
    "required": ["pipelines"],
}
