# 笔记

- 定长数组Array
- 变长数组ArrayBuffer
- 数组转换：守卫和yield关键字
    
    val new_elem = for( elem <- a if elem % 2 == 0 ) yield 2 * elem
- until关键字和to关键字
- 常用Array接口
    - indices方法
    - max、min、sum
    - sortWith、sorted
    - mkString（"<"," ",">"）
- 使用Array.ofDim[]构建多维数组
- 通过导入scala.collection.JavaConversions._进行自动的Java类型转换