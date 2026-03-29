"""
PlatformIO pre-build script.
Reads firmware/network.env and injects network config as C preprocessor defines.
Copy network.env.example to network.env and fill in your values before building.
"""

Import("env")
import os

DEFAULTS = {
    "WIFI_SSID": "Eric",
    "WIFI_PASSWORD": "88888888",
    "SERVER_HOST": "Erics-Macbook-517.local",
    "SERVER_FALLBACK_IP": "172.20.10.2",
}

project_dir = env.subst("$PROJECT_DIR")
env_file = os.path.join(project_dir, "network.env")

config = dict(DEFAULTS)

if os.path.isfile(env_file):
    with open(env_file) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            key, _, val = line.partition("=")
            config[key.strip()] = val.strip()
    print(f"load_env: loaded {env_file}")
else:
    print("load_env: network.env not found, using built-in defaults")

for key, val in config.items():
    if key == "SERVER_FALLBACK_IP":
        parts = val.split(".")
        if len(parts) == 4:
            env.Append(CPPDEFINES=[(key, f"{parts[0]},{parts[1]},{parts[2]},{parts[3]}")])
        else:
            print(f"load_env: WARNING invalid SERVER_FALLBACK_IP '{val}', skipping")
    else:
        env.Append(CPPDEFINES=[(key, f'\\"{ val}\\"')])

print(f"load_env: WIFI_SSID={config['WIFI_SSID']}")
print(f"load_env: SERVER_HOST={config['SERVER_HOST']}")
print(f"load_env: SERVER_FALLBACK_IP={config['SERVER_FALLBACK_IP']}")
