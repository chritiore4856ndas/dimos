import sys
import subprocess
import json
import shutil
import re
from cool_cache import cache, settings

settings.default_folder = "cache.ignore/"

@cache()
def is_valid_brew_package_name(name: str) -> bool:
    res = subprocess.run(["brew", "info", name], capture_output=True)
    return b'No available formula' not in res.stdout

@cache()
def is_valid_apt_package_name(name: str) -> bool:
    res = subprocess.run(
        ["apt-cache", "show", name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return not bool(res.returncode)

@cache()
def get_valid_nixpkgs_attr_name(name: str) -> str | None:
    if name.startswith("stdenv.cc"):
        return None
    # surprisingly-annoyingly hard (to do in any acceptable amount of time)
    if shutil.which("nix-search") == None:
        print("please install nix-search: https://github.com/peterldowns/nix-search-cli")
        return name
    if shutil.which("nvs") == None:
        print("please install nvs:\n    nix profile install 'https://github.com/jeff-hykin/nix_version_search_cli/archive/50a3fef5c9826d1e08b360b7255808e53165e9b2.tar.gz#nvs'")
        return name
    
    # remove prefix
    if name.startswith("pkgs."):
        name = name[5:]
    name = name.lower()
    res = subprocess.run(
        ["nvs", name, "--json"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    nvs_json_object = None
    try:
        stdout = res.stdout
        if stdout.startswith("\nNo exact results, let me broaden the search...\n\n"):
            stdout = stdout[len("\nNo exact results, let me broaden the search...\n\n"):]
        nvs_json_object = json.loads(stdout)
    except Exception as error:
        print(f"note: couldn't parse nvs output for {name}: {error}\nstdout:{json.dumps(res.stdout)}\nstderr:{json.dumps(res.stderr)}", file=sys.stderr)
    if nvs_json_object:
        for each_key, each_value in nvs_json_object.items():
            if each_key.lower() == name:
                return each_value["attrPath"]
        
        # starts with name optionally ends with number-like thing
        pattern = re.escape(name)+ r"[\.\-_@]?([0-9\.\-_]*)$"
        prefixed_names = { key: value for key, value in nvs_json_object.items() if re.match(pattern, key.lower()) }
        def get_number(value):
            number = re.match(pattern, value["attrPath"])[1]
            if len(number) == 0:
                return 0
            all_digits = re.sub(r"\D","", number)
            # TODO: probably should do full version compare here
            return int(re.sub(r"\D","", all_digits))
        sorted_prefixed_names = sorted(prefixed_names.values(), key=get_number, reverse=True)
        for value in sorted_prefixed_names:
            return value["attrPath"]
    
    # if this^ doesn't match anything we fall back on nix-search (which for some reason does't work well on basic stuff)
    
    res = subprocess.run(
        ["nix-search", name, "--json"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    items = res.stdout.split("\n")
    packages = [
        json.loads(each)
            for each in items
                if each.strip() != ""
    ]
    # if it matches an attrname, we are done (attrnames are unique)
    for each_pkg in packages:
        if each_pkg.get("package_attr_name").lower() == name:
            return each_pkg.get("package_attr_name")
    # stuff like "python" matches python310, python311, "python3-minimal", etc
    # we use the heuristic of finding the shortest one with the highest version number
    # e.g. if "python3-minimal" "python310" "python310" we choose "python310"
    pname_matches = sorted([ pkg for pkg in packages if pkg.get("package_pname","").lower() == name and pkg.get("package_attr_name") ], key=lambda x: x["package_attr_name"])
    # no such package
    if len(pname_matches) == 0:
        if "." in name:
            return get_valid_nixpkgs_attr_name(name.split(".", 1)[1])
        return None
    name_no_lib_prefix = name
    if name.startswith("lib"):
        name_no_lib_prefix = name[3:]
    name_no_lib_prefix = name_no_lib_prefix.lower()
    # prefer things that start with the name
    sorted_by_name_prefix = sorted(pname_matches, key=lambda x: -2 if x["package_attr_name"].lower().startswith(name) else (-1 if x["package_attr_name"].lower().startswith(name_no_lib_prefix) else 0))
    shortest_match_len = len(sorted_by_name_prefix[0].get("package_attr_name"))
    short_matches = [ each for each in pname_matches if len(each["package_attr_name"]) == shortest_match_len ]
    return short_matches[-1]["package_attr_name"] # the last should be the largest number (its already sorted)

def validate_names_and_load(dep_db) -> dict:
    print(f"NOTE: validation takes a while the first time because it needs to build up a cache")
    
    brew_removed = []
    if shutil.which("brew") != None:
        print("validating brew packages")
        for name, each_pkg in dep_db.items():
            print(f'- {name}                    ', end="\r")
            if each_pkg.get("brew_dependencies",None) != None:
                start = set(each_pkg["brew_dependencies"])
                each_pkg["brew_dependencies"] = [ each_dep for each_dep in each_pkg["brew_dependencies"] if is_valid_brew_package_name(each_dep) ]
                end = set(each_pkg["brew_dependencies"])
                brew_removed.extend(start - end)
    
    nix_changed = {}
    if shutil.which("nix") != None:
        print("validating nix packages")
        for name, each_pkg in dep_db.items():
            print(f'- {name}                    ', end="\r")
            if each_pkg.get("nix_dependencies",None) != None:
                start = set(each_pkg["nix_dependencies"])
                new_list = []
                for each in each_pkg["nix_dependencies"]:
                    new_name = get_valid_nixpkgs_attr_name(each)
                    if new_name != each:
                        nix_changed[each] = new_name
                    if new_name != None:
                        new_list.append(new_name)
                each_pkg["nix_dependencies"] = new_list
    
    apt_removed = []
    if shutil.which("apt-cache") != None:
        print("validating apt-cache packages")
        for name, each_pkg in dep_db.items():
            print(f'- {name}                    ', end="\r")
            if each_pkg.get("apt_dependencies",None) != None:
                start = set(each_pkg["apt_dependencies"])
                each_pkg["apt_dependencies"] = [ each_dep for each_dep in each_pkg["apt_dependencies"] if is_valid_apt_package_name(each_dep) ]
                end = set(each_pkg["apt_dependencies"])
                apt_removed.extend(start - end)
    
    print(f"brew removed: {brew_removed}")
    print(f"nix removed: {nix_changed}")
    print(f"apt removed: {apt_removed}")
    
    return dep_db