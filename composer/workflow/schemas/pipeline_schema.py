#
#  Copyright (c) 2025 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
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
