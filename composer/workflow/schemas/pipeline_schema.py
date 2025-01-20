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
