import json
import unittest

from ros2_ui.domain.Project import ProjectInfo, ProjectDependencies, Project


class TestCaseProject(unittest.TestCase):
    def setUp(self):
        self.projectInfo = ProjectInfo(
            package_name="Test Package",
            license="My License",
            version="1.2.3",
            maintainer_mail="mail@example.com",
            maintainer="Maint Ainer",
            description="Description of the package 'Test Package'"
        )

        self.projectDependencies = ProjectDependencies(
            exec_depends=["exec_dep_1", "exec_dep_2"],
            test_depends=["test_dep_1", "test_dep_2"]
        )

        self.project = Project(
            project_info=self.projectInfo,
            project_dependencies=self.projectDependencies,
            pubs=[],
            subs=[]
        )

    def test_Project_ProjectInfo_ValueAssignment(self):
        self.assertEqual(self.project.project_info.package_name, "Test Package")
        self.assertEqual(self.project.project_info.license, "My License")
        self.assertEqual(self.project.project_info.version, "1.2.3")
        self.assertEqual(self.project.project_info.maintainer_mail, "mail@example.com")
        self.assertEqual(self.project.project_info.maintainer, "Maint Ainer")
        self.assertEqual(self.project.project_info.description, "Description of the package 'Test Package'")
        self.assertEqual(self.project.project_info, self.projectInfo)

    def test_Project_ProjectDependencies_ValueAssignment(self):
        self.assertEqual(self.projectDependencies.exec_depends, ["exec_dep_1", "exec_dep_2"])
        self.assertEqual(self.projectDependencies.test_depends, ["test_dep_1", "test_dep_2"])
        self.assertEqual(self.project.project_dependencies, self.projectDependencies)

    def test_Project_ProjectInfo_from_dict(self):
        init_dict = {
            "package_name": "TestPackage",
            "license": "My License",
            "version": "1.2.3",
            "maintainer_mail": "mail@example.com",
            "maintainer": "Maint Ainer",
            "description": "Description of the package 'Test Package'"
        }
        project_info = ProjectInfo.from_dict(init_dict)

        self.assertDictEqual(init_dict, project_info.to_dict())

    def test_Project_ProjectDependencies_from_dict(self):
        init_dict = {
            "exec_depends": ["exec_dep_1", "exec_dep_2"],
            "test_depends": ["test_dep_1", "test_dep_2"]
        }
        project_dependencies = ProjectDependencies.from_dict(init_dict)

        self.assertDictEqual(init_dict, project_dependencies.to_dict())

    def test_Project_from_dict(self):
        init_dict = {
            'project_info': {
                'package_name': 'TestPackage',
                'version': '1.2.3',
                'description': "Description of the package 'Test Package'",
                'maintainer_mail': 'mail@example.com',
                'maintainer': 'Maint Ainer',
                'license': 'My License'
            },
            'project_dependencies': {
                'test_depends': ['test_dep_1', 'test_dep_2'],
                'exec_depends': ['exec_dep_1', 'exec_dep_2']
            },
            'pubs': [],
            'subs': []
        }
        project = Project.from_dict(init_dict)
        print(json.dumps(project.to_dict(), indent=2))

        self.assertDictEqual(init_dict, project.to_dict())


if __name__ == '__main__':
    unittest.main()
