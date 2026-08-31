function manifest = packageACCRoadRunnerProject(roadRunnerProject, options)
%PACKAGEACCROADRUNNERPROJECT Create a portable RoadRunner project bundle.
%
%   MANIFEST = PACKAGEACCROADRUNNERPROJECT() packages the project connected
%   through rrApp in the base workspace. If rrApp is unavailable, the
%   original development project is used.
%
%   MANIFEST = PACKAGEACCROADRUNNERPROJECT(PROJECT) accepts either a
%   RoadRunner project folder or its Project/Project.rrproj file.
%
%   The dependency closure for ACC_LeadBrake_Test.rrscenario is copied to
%   the RoadRunnerProject folder beside this function. Unrelated scenes,
%   scenarios, exports, and assets are excluded.

arguments
    roadRunnerProject {mustBeTextScalar} = ""
    options.OutputFolder {mustBeTextScalar} = ...
        fullfile(fileparts(mfilename("fullpath")), "RoadRunnerProject")
end

if strlength(string(roadRunnerProject)) == 0
    roadRunnerProject = defaultProject();
end

sourceRoot = resolveProjectRoot(roadRunnerProject);
outputRoot = char(options.OutputFolder);

if strcmpi(canonicalPath(sourceRoot), canonicalPath(outputRoot))
    error("ACC:BundleInsideSource", ...
        "The bundle output folder must differ from the source project.");
end

scenarioFile = "Scenarios/ACC_LeadBrake_Test.rrscenario";
sceneFile = "Scenes/SixLaneHighway.rrscene";
projectFiles = recursiveFiles(fullfile(sourceRoot, "Project"), sourceRoot);
defaultAssets = readDefaultAssets(fullfile(sourceRoot, ...
    "Project", "DefaultAssets.xml"));
pendingFiles = [scenarioFile; sceneFile; projectFiles; defaultAssets];

[relativeFiles, missingFiles] = dependencyClosure(sourceRoot, pendingFiles);
if ~isempty(missingFiles)
    error("ACC:MissingRoadRunnerDependencies", ...
        "Missing RoadRunner dependencies:\n%s", ...
        strjoin("  " + missingFiles, newline));
end

if ~isfolder(outputRoot)
    mkdir(outputRoot);
end

numFiles = numel(relativeFiles);
bytes = zeros(numFiles, 1);
sha256 = strings(numFiles, 1);
for fileIndex = 1:numFiles
    relativePath = relativeFiles(fileIndex);
    sourceFile = fullfile(sourceRoot, relativePath);
    destinationFile = fullfile(outputRoot, relativePath);
    destinationFolder = fileparts(destinationFile);
    if ~isfolder(destinationFolder)
        mkdir(destinationFolder);
    end

    copied = copyfile(sourceFile, destinationFile, "f");
    assert(copied, "ACC:CopyFailed", ...
        "Could not copy '%s' to the RoadRunner bundle.", sourceFile);

    fileInfo = dir(destinationFile);
    bytes(fileIndex) = fileInfo.bytes;
    sha256(fileIndex) = fileSha256(destinationFile);
end

manifest = table(relativeFiles, bytes, sha256, ...
    VariableNames=["RelativePath", "Bytes", "SHA256"]);
writetable(manifest, fullfile(outputRoot, "BUNDLE_MANIFEST.csv"));

fprintf("Packaged %d files (%.2f MiB) in:\n  %s\n", ...
    height(manifest), sum(manifest.Bytes) / 1024^2, outputRoot);
end

function project = defaultProject()
if evalin("base", "exist('rrApp', 'var')")
    rrApp = evalin("base", "rrApp");
    rrStatus = status(rrApp);
    project = string(rrStatus.Project.Filename);
else
    project = "D:/Work/26a/checkShirt/checkShort";
end
end

function projectRoot = resolveProjectRoot(projectInput)
projectInput = char(projectInput);
if isfile(projectInput)
    [projectFolder, projectName, projectExtension] = fileparts(projectInput);
    if ~strcmpi(projectName + string(projectExtension), "Project.rrproj")
        error("ACC:InvalidRoadRunnerProjectFile", ...
            "Expected a Project.rrproj file, received '%s'.", projectInput);
    end
    projectRoot = fileparts(projectFolder);
elseif isfolder(projectInput)
    if isfile(fullfile(projectInput, "Project", "Project.rrproj"))
        projectRoot = projectInput;
    elseif isfile(fullfile(projectInput, "Project.rrproj"))
        projectRoot = fileparts(projectInput);
    else
        error("ACC:InvalidRoadRunnerProjectFolder", ...
            "'%s' does not contain Project/Project.rrproj.", projectInput);
    end
else
    error("ACC:RoadRunnerProjectNotFound", ...
        "RoadRunner project does not exist: '%s'.", projectInput);
end

projectRoot = canonicalPath(projectRoot);
end

function files = recursiveFiles(folder, projectRoot)
entries = dir(fullfile(folder, "**", "*"));
entries = entries(~[entries.isdir]);
absoluteFiles = string(fullfile({entries.folder}, {entries.name}))';
files = replace(extractAfter(absoluteFiles, strlength(string(projectRoot)) + 1), ...
    "\", "/");
end

function assets = readDefaultAssets(defaultAssetsFile)
fileText = fileread(defaultAssetsFile);
matches = regexp(fileText, 'path="(?<path>Assets/[^"]+)"', "names");
if isempty(matches)
    assets = strings(0, 1);
    return
end

assets = string({matches.path})';
hasExtension = arrayfun(@(pathValue) ...
    strlength(string(filepartsExtension(pathValue))) > 0, assets);
assets = unique(assets(hasExtension), "stable");
end

function extension = filepartsExtension(pathValue)
[~, ~, extension] = fileparts(pathValue);
end

function [files, missing] = dependencyClosure(projectRoot, pendingFiles)
pendingFiles = string(pendingFiles(:));
visited = strings(0, 1);
files = strings(0, 1);
missing = strings(0, 1);
nextFile = 1;

while nextFile <= numel(pendingFiles)
    relativePath = normalizeRelativePath(pendingFiles(nextFile));
    nextFile = nextFile + 1;

    if any(strcmpi(relativePath, visited))
        continue
    end
    visited(end + 1, 1) = relativePath; %#ok<AGROW>

    absolutePath = fullfile(projectRoot, relativePath);
    if ~isfile(absolutePath)
        metadataPath = relativePath + ".rrmeta";
        if ~endsWith(relativePath, ".rrmeta", IgnoreCase=true) ...
                && isfile(fullfile(projectRoot, metadataPath))
            pendingFiles(end + 1, 1) = metadataPath; %#ok<AGROW>
        else
            missing(end + 1, 1) = relativePath; %#ok<AGROW>
        end
        continue
    end

    files(end + 1, 1) = relativePath; %#ok<AGROW>

    if ~endsWith(relativePath, ".rrmeta", IgnoreCase=true)
        metadataPath = relativePath + ".rrmeta";
        if isfile(fullfile(projectRoot, metadataPath))
            pendingFiles(end + 1, 1) = metadataPath; %#ok<AGROW>
        end
    end

    references = readProjectReferences(absolutePath);
    pendingFiles = [pendingFiles; references]; %#ok<AGROW>
end

files = sort(unique(files));
missing = sort(unique(missing));
end

function references = readProjectReferences(filePath)
fileId = fopen(filePath, "rb");
assert(fileId >= 0, "ACC:FileReadFailed", "Could not read '%s'.", filePath);
closeFile = onCleanup(@() fclose(fileId));
fileBytes = fread(fileId, Inf, "*uint8");
fileText = char(fileBytes');

extensions = [ ...
    "rrscenario", "rrbehavior", "rrscene", "rrmtl_rrx", ...
    "rrlms_rrx", "rrcws_rrx", "rrpms_rrx", "fbx_rrx", ...
    "svg_rrx", "rrmeta", "rrmtl", "rrlms", "rrcws", "rrpms", ...
    "rrext", "rrhd", "rrproj", "fbx", "png", "jpeg", "jpg", ...
    "tiff", "tif", "bmp", "svg"];
expression = "<PROJECT>[\\/](?<path>[ -~]+?\.(?:" ...
    + strjoin(extensions, "|") + "))";
matches = regexp(fileText, expression, "names", "ignorecase");

if isempty(matches)
    references = strings(0, 1);
else
    references = unique(normalizeRelativePath(string({matches.path})'));
end

clear closeFile
end

function pathValue = normalizeRelativePath(pathValue)
pathValue = replace(string(pathValue), "\", "/");
pathValue = regexprep(pathValue, "^\./", "");
end

function pathValue = canonicalPath(pathValue)
pathValue = char(java.io.File(char(pathValue)).getCanonicalPath());
end

function checksum = fileSha256(filePath)
fileId = fopen(filePath, "rb");
assert(fileId >= 0, "ACC:FileReadFailed", "Could not read '%s'.", filePath);
closeFile = onCleanup(@() fclose(fileId));
fileBytes = fread(fileId, Inf, "*uint8");

digest = java.security.MessageDigest.getInstance("SHA-256");
digest.update(fileBytes);
hashBytes = typecast(digest.digest(), "uint8");
checksum = lower(string(reshape(dec2hex(hashBytes, 2).', 1, [])));

clear closeFile
end
