#include "raw_mesh.h"
#include <fstream>

raw_mesh::raw_mesh(const std::string & path)
{
	std::ifstream file;
	file.open(path);
	assert(file.is_open());
	std::string line;
	while (std::getline(file, line))
	{
		if (line.size() == 0U)
			continue;
		if (line.rfind("v ") == 0)
		{
			glm::vec3 v;
			line = line.substr(2);
			size_t s = line.find(' ');
			v[0] = static_cast<float>(std::atof(line.substr(0, s).c_str()));
			line = line.substr(s + 1);
			s = line.find(' ');
			v[1] = static_cast<float>(std::atof(line.substr(0, s).c_str()));
			line = line.substr(s + 1);
			s = line.find(' ');
			v[2] = static_cast<float>(std::atof(line.substr(0, s).c_str()));
			m_vertices.push_back(v);
		}
		else if (line.rfind("f ") == 0)
		{
			std::vector<uint> f;
			line = line.substr(2);
			while (line.size() > 0)
			{
				size_t b = line.find('/');
				size_t s = line.find(' ');
				if (s > line.length()) s = line.length() - 1;
				if (b > s) b = s+1;

				int i = std::atoi(line.substr(0, b).c_str()) - 1;
				f.push_back(static_cast<uint>(i));
				line = line.substr(s + 1);

			}
			m_faces.push_back(f);
		}
	}
	file.close();
}
