# templates书写格式
templates文件的格式依照[Go Template Lang](https://golang.org/pkg/text/template/),并且整合了一些[Sprig函数](https://github.com/Masterminds/sprig)和其他[专用函数](https://github.com/helm/helm/blob/master/docs/charts_tips_and_tricks.md)

# 预定义的Values

以下Values是预定义的且不能被覆盖！

参考：https://github.com/helm/helm/blob/master/docs/charts.md#predefined-values

需要注意的是：一些的文件可以在Chart的目录下，并使用{{index .Files "file.name"}}、{{.Files.Get name}}、{{.Files.GetString name}}等方式访问其内容。

# Value.xml

- 在使用Helm install安装Chart包时，除了使用目录内的values.yaml提供默认配置，还可以使用--values=myvals.yaml指定特殊的配置文件。多个values.yaml会合并成一个，--values指定的覆盖chart目录内的。

- values.yaml可以为子Chart提供值，父Chart也可以访问子Chart的values.yaml值（.Values.ChildChart.key），但是子Chart不能访问父Chart的Value

- global空间内的值，所有Chart都能访问{{.Values.global.key}}，父Chart的global覆盖子chart



# 参考

https://godoc.org/github.com/Masterminds/sprig
http://yaml.org/spec/